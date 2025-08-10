#!/usr/bin/env python3

import rospy as ros
from std_msgs.msg import Int32

from autonomous_locking.msg import Target


class LockManager:
    def __init__(self):
        ros.init_node("lock_manager")

        self.locked_target = None
        self.lock_start_time = None
        self.lock_duration_required = float(ros.get_param("~lock_duration", 4.0))
        self.lock_confirm_pub = ros.Publisher(
            "/autonomous_locking/lock_confirm", Target, queue_size=1
        )
        self.exclude_pub = ros.Publisher(
            "/autonomous_locking/locked_out_target_id", Int32, queue_size=1
        )

        ros.Subscriber("/autonomous_locking/best_target", Target, self.target_callback)
        ros.Timer(ros.Duration(0.5), self.check_lock_status)

    def target_callback(self, msg: Target):
        if self.locked_target is None or self.locked_target.id != msg.id:
            ros.loginfo(f"[Kilit] Yeni hedefe geçiliyor: ID={msg.id}")
            self.locked_target = msg
            self.lock_start_time = ros.Time.now()
        else:
            ros.loginfo_throttle(1.0, f"[Kilit] Hedef izleniyor: ID={msg.id}")

    def check_lock_status(self, event):
        if self.locked_target is None:
            return

        elapsed = (ros.Time.now() - self.lock_start_time).to_sec()
        if elapsed >= self.lock_duration_required:
            ros.loginfo(
                f"[Kilit] {self.locked_target.id} ID'li hedef {elapsed:.1f} saniye boyunca kilitlendi."
            )
            self.on_lock_complete(self.locked_target)
            self.locked_target = None
            self.lock_start_time = None

    def on_lock_complete(self, target: Target):
        self.lock_confirm_pub.publish(target)
        self.exclude_pub.publish(target.id)
        ros.loginfo(f"[Kilit] Hedef başarıyla kilitlendi: ID={target.id}")


if __name__ == "__main__":
    try:
        LockManager()
        ros.spin()
    except ros.ROSInterruptException:
        pass
