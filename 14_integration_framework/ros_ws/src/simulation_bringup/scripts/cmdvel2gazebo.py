#!/usr/bin/env python3
"""Translate Twist commands into low-level wheel/steer commands."""

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVel2Gazebo:
    def __init__(self) -> None:
        rospy.init_node("cmdvel2gazebo", anonymous=True)

        self.robot_name = rospy.get_param("~robot_name", "smart")
        self.cmd_topic = rospy.get_param("~cmd_topic", f"/{self.robot_name}/cmd_vel")
        controller_ns = rospy.get_param("~controller_namespace", f"/{self.robot_name}")

        self.pub_steerL = rospy.Publisher(f"{controller_ns}/front_left_steering_position_controller/command", Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher(f"{controller_ns}/front_right_steering_position_controller/command", Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher(f"{controller_ns}/rear_left_velocity_controller/command", Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher(f"{controller_ns}/rear_right_velocity_controller/command", Float64, queue_size=1)

        rospy.Subscriber(self.cmd_topic, Twist, self.callback, queue_size=1)

        self.x = 0.0
        self.z = 0.0
        self.L = rospy.get_param("~wheelbase", 1.868)
        self.T_front = rospy.get_param("~front_track", 1.284)
        self.T_rear = rospy.get_param("~rear_track", 1.284)

        timeout_sec = rospy.get_param("~timeout", 0.2)
        self.timeout = rospy.Duration.from_sec(timeout_sec)
        self.last_msg = rospy.Time.now()

        max_inside = rospy.get_param("~max_inside", 0.6)
        r_max = self.L / math.tan(max_inside)
        r_ideal = r_max + (self.T_front / 2.0)
        self.maxsteer = math.atan2(self.L, r_ideal)

        self.rate = rospy.Rate(rospy.get_param("~rate", 10))

    def callback(self, msg: Twist) -> None:
        self.x = msg.linear.x / 0.3
        self.z = max(-self.maxsteer, min(self.maxsteer, msg.angular.z))
        self.last_msg = rospy.Time.now()

    def publish(self) -> None:
        delta = rospy.Time.now() - self.last_msg
        if delta > self.timeout:
            self._stop_motion()
            return

        if self.z != 0:
            r = self.L / math.fabs(math.tan(self.z))
            rL_rear = r - (math.copysign(1, self.z) * (self.T_rear / 2.0))
            rR_rear = r + (math.copysign(1, self.z) * (self.T_rear / 2.0))
            rL_front = r - (math.copysign(1, self.z) * (self.T_front / 2.0))
            rR_front = r + (math.copysign(1, self.z) * (self.T_front / 2.0))

            msgRearR = Float64(data=self.x * rR_rear / r)
            msgRearL = Float64(data=self.x * rL_rear / r)
            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64(data=math.atan2(self.L, rL_front) * math.copysign(1, self.z))
            msgSteerR = Float64(data=math.atan2(self.L, rR_front) * math.copysign(1, self.z))
            self.pub_steerL.publish(msgSteerL)
            self.pub_steerR.publish(msgSteerR)
        else:
            msgRear = Float64(data=self.x)
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            msgSteer = Float64(data=self.z)
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)

    def _stop_motion(self) -> None:
        stop = Float64(data=0.0)
        self.pub_rearL.publish(stop)
        self.pub_rearR.publish(stop)
        self.pub_steerL.publish(stop)
        self.pub_steerR.publish(stop)

    def spin(self) -> None:
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = CmdVel2Gazebo()
        node.spin()
    except rospy.ROSInterruptException:
        pass
