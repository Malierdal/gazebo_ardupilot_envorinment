#!/usr/bin/env python3

import math

import rospy as ros
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import AttitudeTarget

from autonomous_locking.msg import Target


class ApproachController:
    def __init__(self):
        ros.init_node("approach_controller")

        self.target = None
        self.own_pose = None
        self.start_time = None

        self.att_pub = ros.Publisher(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1
        )

        ros.Subscriber("/autonomous_locking/best_target", Target, self.target_callback)
        ros.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.own_pose_callback
        )

        ros.Timer(ros.Duration(0.1), self.update)

        self.R0 = ros.get_param("~initial_radius", 25.0)
        self.lambda_decay = ros.get_param("~radius_decay", 0.2)
        self.omega = ros.get_param("~angular_speed", 0.5)
        self.thrust = ros.get_param("~spiral_thrust", 0.6)

    def target_callback(self, msg: Target):
        self.target = msg
        self.start_time = ros.Time.now()

    def own_pose_callback(self, msg: PoseStamped):
        self.own_pose = msg.pose

    def update(self, event):
        if self.own_pose is None or self.target is None or self.start_time is None:
            return

        t = (ros.Time.now() - self.start_time).to_sec()
        if t > 15.0:
            return

        tx = self.target.pose.position.x
        ty = self.target.pose.position.y

        r = self.R0 * math.exp(-self.lambda_decay * t)
        theta = self.omega * t

        spiral_x = tx - r * math.cos(theta)
        spiral_y = ty - r * math.sin(theta)

        dx = spiral_x - self.own_pose.position.x
        dy = spiral_y - self.own_pose.position.y

        desired_yaw = math.atan2(dy, dx)
        quat = self.yaw_to_quaternion(desired_yaw)

        msg = AttitudeTarget()
        msg.type_mask = 7
        msg.orientation = quat
        msg.thrust = self.thrust

        self.att_pub.publish(msg)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0
        q.y = 0
        q.z = math.sin(yaw / 2)
        return q


if __name__ == "__main__":
    try:
        ApproachController()
        ros.spin()
    except ros.ROSInterruptException:
        pass
        ros.spin()
    except ros.ROSInterruptException:
        pass
