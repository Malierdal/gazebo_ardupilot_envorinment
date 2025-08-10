#!/usr/bin/env python3

import math

import rospy as ros
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from std_msgs.msg import Int32
from utils import compute_zone_score, load_cost_weights, set_zones

from autonomous_locking.msg import Target

EXCLUSION_DURATION = 10.0


class TargetSelectorTF:
    def __init__(self):
        ros.init_node("target_selector_tf")
        load_cost_weights()

        self.locked_out_target_id = None
        self.locked_out_timestamp = None
        self.targets = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.best_target_pub = ros.Publisher(
            "/autonomous_locking/best_target", Target, queue_size=1
        )

        ros.Subscriber("/autonomous_locking/targets", Target, self.target_callback)
        ros.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.own_pose_callback
        )
        ros.Subscriber(
            "/autonomous_locking/locked_out_target_id", Int32, self.exclude_callback
        )
        ros.Subscriber("/nfz_coords", PointStamped, self.zone_callback)

        self.own_pose = None

        ros.Timer(ros.Duration(1.0), self.evaluate_targets)

    def own_pose_callback(self, msg: PoseStamped):
        self.own_pose = msg

    def target_callback(self, msg: Target):
        msg.header.stamp = ros.Time.now()
        self.targets[msg.id] = msg

    def exclude_callback(self, msg: Int32):
        self.locked_out_target_id = msg.data
        self.locked_out_timestamp = ros.Time.now()
        ros.loginfo(
            f"[HedefSeçici] Yakın zamanda kilitlenilmiş hedef ID={msg.data} görmezden geliniyor."
        )

    def zone_callback(self, msg: PointStamped):
        set_zones([msg.point])

    def angle_diff(self, a, b):
        d = abs((a - b + 180) % 360 - 180)
        return d

    def evaluate_targets(self, _):
        if self.own_pose is None:
            ros.logwarn_throttle(5, "[HedefSeçici] Uçağın konumu alınamadı.")
            return
        if not self.targets:
            ros.logwarn_throttle(5, "[HedefSeçici] Herhangi bir hedeef bulunamadı.")
            return

        best_target = None
        best_cost = float("inf")
        now = ros.Time.now()

        for tid, target in self.targets.items():
            if (now - target.header.stamp).to_sec() > 3.0:
                continue

            if self.locked_out_target_id == target.id:
                if (now - self.locked_out_timestamp).to_sec() < EXCLUSION_DURATION:
                    ros.loginfo_throttle(
                        10, f"[HedefSeçici] Hedef ID={tid} görmezden geliniyor."
                    )
                    continue

            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", f"target_{tid}", ros.Time(0), ros.Duration(0.1)
                )

                dx = tf.transform.translation.x
                dy = tf.transform.translation.y
                dz = tf.transform.translation.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)

                angle_to_target = math.degrees(math.atan2(dy, dx))

                tail_angle = self.angle_diff(angle_to_target, target.heading)

                own_heading = self.get_heading_from_quaternion(
                    self.own_pose.pose.orientation
                )
                heading_alignment = self.angle_diff(own_heading, target.heading)

                zone_score = compute_zone_score(target)

                norm_d = distance / 1000.0
                norm_tail = tail_angle / 180.0
                norm_align = heading_alignment / 180.0

                from utils import WEIGHTS

                cost = (
                    WEIGHTS.get("distance") * norm_d
                    + WEIGHTS.get("tail_angle") * norm_tail
                    + WEIGHTS.get("heading_alignment") * norm_align
                    + WEIGHTS.get("zone_score") * zone_score
                )

                target.distance = distance
                target.tail_angle = tail_angle
                target.heading_alignment = heading_alignment
                target.zone_score = zone_score
                target.total_cost = cost

                if cost < best_cost:
                    best_cost = cost
                    best_target = target

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                ros.logwarn_throttle(5.0, f"[TF] hedef_{tid} için dönüşüm hatası: {e}")
                continue

        if best_target:
            self.best_target_pub.publish(best_target)
            ros.loginfo_throttle(
                2.0,
                f"[HedefSeçici] En iyi hedef: ID={best_target.id}, Maliyet={best_target.total_cost:.3f}",
            )

    def get_heading_from_quaternion(self, q):
        import tf.transformations as tft

        euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = math.degrees(euler[2])
        return yaw % 360


if __name__ == "__main__":
    try:
        TargetSelectorTF()
        ros.spin()
    except ros.ROSInterruptException:
        pass
