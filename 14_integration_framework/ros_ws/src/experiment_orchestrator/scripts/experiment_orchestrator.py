#!/usr/bin/env python3
"""High-level orchestration for experiments combining environment, vehicle and controller."""

import os
from pathlib import Path
from typing import Dict

import rospy
import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class Orchestrator:
    def __init__(self) -> None:
        pkg_root = Path(__file__).resolve().parents[1]
        default_cfg = pkg_root / "config" / "experiments.yaml"
        self.cfg_path = Path(rospy.get_param("~config_file", str(default_cfg))).expanduser()
        self.log_root = None
        self.experiments: Dict[str, Dict] = {}
        self.current_id: str = ""

        self.vehicle_pub = rospy.Publisher("/simulation_bringup/environment_loader/set_vehicle", String, queue_size=1, latch=True)
        self.world_pub = rospy.Publisher("/simulation_bringup/environment_loader/set_world", String, queue_size=1, latch=True)
        self.controller_pub = rospy.Publisher("/algorithm_manager/algorithm_switcher/set_active", String, queue_size=1, latch=True)
        self.exp_pub = rospy.Publisher("~active_experiment", String, queue_size=1, latch=True)

        self.reload_srv = rospy.Service("~reload", Trigger, self._handle_reload)
        rospy.Subscriber("~set_experiment", String, self._handle_set_experiment)

        self._load_config()
        initial = rospy.get_param("~experiment_id", next(iter(self.experiments), None))
        if initial:
            self.apply(initial)
        rospy.loginfo("experiment_orchestrator ready; total experiments: %d", len(self.experiments))

    def _load_config(self) -> None:
        fallback = Path(__file__).resolve().parents[5] / "logs"
        fallback.mkdir(parents=True, exist_ok=True)
        if not self.cfg_path.exists():
            rospy.logwarn("Experiment config %s not found", self.cfg_path)
            self.experiments = {}
            self.log_root = fallback
            return
        with self.cfg_path.open("r", encoding="utf-8") as handler:
            data = yaml.safe_load(handler) or {}
        self.experiments = data.get("experiments", {})
        root = data.get("log_root", "logs")
        expanded = os.path.expandvars(root.replace("${REPO_ROOT}", str(self._repo_root())))
        self.log_root = Path(expanded).expanduser()
        self.log_root.mkdir(parents=True, exist_ok=True)

    def _repo_root(self) -> Path:
        return Path(__file__).resolve().parents[6]

    def _handle_reload(self, _: TriggerRequest = None) -> TriggerResponse:
        self._load_config()
        if self.current_id and self.current_id in self.experiments:
            self.apply(self.current_id)
        return TriggerResponse(success=True, message="Experiments reloaded")

    def _handle_set_experiment(self, msg: String) -> None:
        exp_id = msg.data.strip()
        self.apply(exp_id)

    def apply(self, exp_id: str) -> None:
        if exp_id not in self.experiments:
            rospy.logwarn("Experiment %s not defined", exp_id)
            return
        spec = self.experiments[exp_id]
        rospy.loginfo("Applying experiment %s", exp_id)
        self.current_id = exp_id

        # propagate selections
        self.vehicle_pub.publish(String(data=spec.get("vehicle_id", "")))
        self.world_pub.publish(String(data=spec.get("world_id", "")))
        self.controller_pub.publish(String(data=spec.get("controller_id", "")))
        self.exp_pub.publish(String(data=exp_id))

        self._write_metadata(exp_id, spec)
        rospy.set_param("/experiment/current", spec)

    def _write_metadata(self, exp_id: str, spec: Dict) -> None:
        log_dir = self.log_root / exp_id
        log_dir.mkdir(parents=True, exist_ok=True)
        meta_file = log_dir / "metadata.yaml"
        with meta_file.open("w", encoding="utf-8") as handler:
            yaml.safe_dump(spec, handler)
        rospy.loginfo("Wrote metadata to %s", meta_file)

    def spin(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("experiment_orchestrator")
    orchestrator = Orchestrator()
    orchestrator.spin()
