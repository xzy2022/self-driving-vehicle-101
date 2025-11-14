#!/usr/bin/env python3
"""Load vehicle/environment definitions and publish the active selection as parameters + latched topics."""

import os
from pathlib import Path
from typing import Dict

import rospy
import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class EnvironmentLoader:
    def __init__(self) -> None:
        pkg_root = Path(__file__).resolve().parents[1]
        default_vehicle_cfg = pkg_root / "config" / "vehicles.yaml"
        default_env_cfg = pkg_root / "config" / "environments.yaml"
        self.vehicle_cfg = Path(rospy.get_param("~vehicles_file", str(default_vehicle_cfg))).expanduser()
        self.env_cfg = Path(rospy.get_param("~environments_file", str(default_env_cfg))).expanduser()

        self.vehicles: Dict = {}
        self.worlds: Dict = {}

        self.vehicle_pub = rospy.Publisher("~vehicle_profile", String, queue_size=1, latch=True)
        self.world_pub = rospy.Publisher("~world_profile", String, queue_size=1, latch=True)
        self.reload_srv = rospy.Service("~reload", Trigger, self._handle_reload)
        rospy.Subscriber("~set_vehicle", String, self._handle_vehicle)
        rospy.Subscriber("~set_world", String, self._handle_world)

        self._load_configs()

        self.active_vehicle = rospy.get_param("~vehicle_id", next(iter(self.vehicles), None))
        self.active_world = rospy.get_param("~world_id", next(iter(self.worlds), None))
        self.select(self.active_vehicle, self.active_world)

    def _handle_reload(self, _: TriggerRequest = None) -> TriggerResponse:
        self._load_configs()
        return TriggerResponse(success=True, message="simulation_bringup config reloaded")

    def _load_configs(self) -> None:
        self.vehicles = self._read_yaml(self.vehicle_cfg).get("vehicles", {})
        self.worlds = self._read_yaml(self.env_cfg).get("worlds", {})
        rospy.loginfo("simulation_bringup loaded %d vehicles and %d worlds", len(self.vehicles), len(self.worlds))

    @staticmethod
    def _read_yaml(path: Path) -> Dict:
        if not path.exists():
            rospy.logwarn("Configuration %s not found", path)
            return {}
        with path.open("r", encoding="utf-8") as handler:
            return yaml.safe_load(handler) or {}

    def select(self, vehicle_id: str, world_id: str) -> None:
        vehicle = self.vehicles.get(vehicle_id)
        world = self.worlds.get(world_id)
        if not vehicle:
            rospy.logwarn("Vehicle profile %s missing", vehicle_id)
        if not world:
            rospy.logwarn("World profile %s missing", world_id)
        if not vehicle or not world:
            return

        rospy.set_param("/simulation_bringup/vehicle", vehicle)
        rospy.set_param("/simulation_bringup/world", world)
        self.vehicle_pub.publish(String(data=yaml.dump(vehicle)))
        self.world_pub.publish(String(data=yaml.dump(world)))

        rospy.loginfo("Selected vehicle=%s, world=%s", vehicle_id, world_id)

        self.active_vehicle = vehicle_id
        self.active_world = world_id
        os.environ["ACTIVE_VEHICLE"] = vehicle_id
        os.environ["ACTIVE_WORLD"] = world_id

    def _handle_vehicle(self, msg: String) -> None:
        requested = msg.data.strip()
        if requested and requested in self.vehicles:
            self.select(requested, self.active_world)
        else:
            rospy.logwarn("Requested vehicle %s not found", requested)

    def _handle_world(self, msg: String) -> None:
        requested = msg.data.strip()
        if requested and requested in self.worlds:
            self.select(self.active_vehicle, requested)
        else:
            rospy.logwarn("Requested world %s not found", requested)

    def spin(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("environment_loader")
    loader = EnvironmentLoader()
    loader.spin()
