#!/usr/bin/env python3

import rospy as ros
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from autonomous_locking.msg import Target

from scripts import auth_manager, telemetry, time_sync


def main():
    ros.init_node("server_interface")

    ros.loginfo("[Sunucu Arayüzü] Başlatılıyor...")

    if not auth_manager.login():
        ros.logwarn(
            "[Sunucu Arayüzü] Otomatik giriş başarısız. /force_login servisiyle tekrar denenebilir."
        )

    auth_manager.init_auth_service()

    telemetry.Target = Target
    telemetry.target_pub = ros.Publisher("/targets", Target, queue_size=10)
    telemetry.start_telemetry()

    time_sync.start_time_sync(interval_sec=10.0)

    ros.loginfo("[Sunucu Arayüzü] Tüm hizmetler başlatıldı.")
    ros.spin()


if __name__ == "__main__":
    main()
