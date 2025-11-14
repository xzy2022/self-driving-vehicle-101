#!/usr/bin/env python3
"""Utility node that keeps track of the currently active trajectory tracking algorithm."""

import os
import signal
import subprocess
from pathlib import Path
from typing import Dict, Optional

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class AlgorithmSwitcher:
    """Load controller definitions from YAML and expose runtime switching hooks."""

    def __init__(self) -> None:
        self.repo_root = Path(os.environ.get("SELF_DRIVING_REPO_ROOT", self._detect_repo_root()))
        self.controllers = {}
        self.current_id: Optional[str] = None
        self.child: Optional[subprocess.Popen] = None

        default_cfg = Path(__file__).resolve().parents[1] / "config" / "controllers.yaml"
        cfg_path = Path(rospy.get_param("~config_file", str(default_cfg))).expanduser()
        self._load_config(cfg_path)

        self.status_pub = rospy.Publisher("~active_controller", String, queue_size=1, latch=True)
        self.diag_pub = rospy.Publisher("~diagnostics", DiagnosticStatus, queue_size=1, latch=True)
        rospy.Subscriber("~set_active", String, self._handle_switch_request)
        self.reload_srv = rospy.Service("~reload_config", Trigger, self._handle_reload)

        initial = rospy.get_param("~active_controller", self.default_controller)
        self.activate(initial)

        rospy.loginfo("algorithm_manager ready; listening for controller switch requests.")

    @staticmethod
    def _detect_repo_root() -> str:
        return str(Path(__file__).resolve().parents[6])

    def _load_config(self, cfg_path: Path) -> None:
        if not cfg_path.exists():
            rospy.logwarn("Controller config %s not found; using empty registry", cfg_path)
            self.controllers = {}
            self.default_controller = None
            return
        with cfg_path.open("r", encoding="utf-8") as handler:
            data = yaml.safe_load(handler) or {}
        self.controllers = data.get("controllers", {})
        self.default_controller = data.get("default_controller")
        rospy.loginfo("Loaded %d controller definitions from %s", len(self.controllers), cfg_path)

    def _handle_reload(self, _: TriggerRequest = None) -> TriggerResponse:
        cfg_path = Path(rospy.get_param("~config_file")).expanduser()
        self._load_config(cfg_path)
        if self.current_id not in self.controllers and self.controllers:
            rospy.logwarn("Previously active controller missing in config, switching to default")
            self.activate(self.default_controller)
        return TriggerResponse(success=True, message="Controller config reloaded")

    def _handle_switch_request(self, msg: String) -> None:
        self.activate(msg.data.strip())

    def activate(self, controller_id: Optional[str]) -> None:
        if not controller_id:
            rospy.logwarn("No controller id provided; ignoring switch request")
            return
        if controller_id not in self.controllers:
            rospy.logwarn("Unknown controller %s", controller_id)
            return
        if controller_id == self.current_id:
            rospy.loginfo("Controller %s already active", controller_id)
            return

        rospy.loginfo("Switching controller from %s to %s", self.current_id, controller_id)
        self._stop_child()

        spec: Dict = self.controllers[controller_id]
        params = spec.get("default_params", {})
        rospy.set_param("~active_controller/name", controller_id)
        rospy.set_param("~active_controller/params", params)

        self._maybe_launch(spec, controller_id)

        self.current_id = controller_id
        self.status_pub.publish(String(data=controller_id))
        self._publish_diag(spec)

    def _maybe_launch(self, spec: Dict, controller_id: str) -> None:
        launch_file = spec.get("launch_file")
        if not launch_file:
            rospy.loginfo("Controller %s has no launch file; expecting external launch manager", controller_id)
            return

        expanded = os.path.expandvars(launch_file.replace("${REPO_ROOT}", str(self.repo_root)))
        expanded = os.path.expanduser(expanded)
        cmd = ["roslaunch"]
        if expanded.endswith(".launch"):
            if " " in expanded:
                cmd.extend(expanded.split())
            elif os.path.isfile(expanded):
                cmd.extend([expanded])
            else:
                # allow package format: roslaunch pkg file.launch
                parts = expanded.split("/")
                cmd.extend(parts[-2:])
        else:
            cmd.extend(spec.get("roslaunch_args", []))
            cmd.append(expanded)

        env = os.environ.copy()
        env["SELF_DRIVING_REPO_ROOT"] = str(self.repo_root)
        env.update({str(k): str(v) for k, v in spec.get("env", {}).items()})

        rospy.loginfo("Launching controller via: %s", " ".join(cmd))
        try:
            self.child = subprocess.Popen(cmd, env=env)
        except (OSError, FileNotFoundError) as err:
            rospy.logerr("Failed to start controller launch: %s", err)
            self.child = None

    def _stop_child(self) -> None:
        if not self.child:
            return
        rospy.loginfo("Stopping previously launched algorithm")
        self.child.send_signal(signal.SIGINT)
        try:
            self.child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            rospy.logwarn("Child process did not exit after SIGINT; sending SIGTERM")
            self.child.terminate()
        self.child = None

    def _publish_diag(self, spec: Dict) -> None:
        status = DiagnosticStatus()
        status.name = "algorithm_manager"
        status.level = DiagnosticStatus.OK
        status.message = f"Active controller: {self.current_id}"
        status.values = [KeyValue(key=k, value=str(v)) for k, v in spec.get("default_params", {}).items()]
        self.diag_pub.publish(status)

    def spin(self) -> None:
        rospy.spin()
        self._stop_child()


if __name__ == "__main__":
    rospy.init_node("algorithm_switcher")
    manager = AlgorithmSwitcher()
    manager.spin()
