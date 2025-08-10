#!/usr/bin/env python3

from datetime import datetime, timezone
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rospy as ros

from autonomous_locking.msg import Target

from scripts.auth_manager import AUTH_STATE
from scripts.common import post_json

now = datetime.now(timezone.utc)


def build_telemetry_packet():
    return {
        "takim_numarasi": AUTH_STATE["team_id"],
        "iha_enlem": 0,
        "iha_boylam": 0,
        "iha_irtifa": 0,
        "iha_dikilme": 0,
        "iha_yonelme": 0,
        "iha_yatis": -0,
        "iha_hiz": 0,
        "iha_batarya": 0,
        "iha_otonom": 1,
        "iha_kilitlenme": 0,
        "hedef_merkez_X": 0,
        "hedef_merkez_Y": 0,
        "hedef_genislik": 0,
        "hedef_yukseklik": 0,
        "gps_saati": {
            "saat": now.hour,
            "dakika": now.minute,
            "saniye": now.second,
            "milisaniye": int(now.microsecond / 1000),
        },
    }


def send_telemetry(event):
    if not AUTH_STATE["logged_in"]:
        ros.logwarn_throttle(
            5.0, "[Telemetri] Telemetri gönderimi atlanıyor: Giriş yapılmamış."
        )
        return

    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/telemetri_gonder"
    payload = build_telemetry_packet()
    status, response = post_json(url, payload)

    if status != 200:
        ros.logwarn_throttle(5.0, f"[Telemetri] Sunucu hata kodu döndürdü: {status}")
        return

    if "konumBilgileri" not in response:
        ros.logwarn("[Telemetri] 'konumBilgileri' eksik.")
        return

    for uav in response["konumBilgileri"]:
        try:
            if uav["takim_numarasi"] == AUTH_STATE["team_id"]:
                continue

            msg = Target()
            msg.id = uav["takim_numarasi"]
            msg.pose.position.x = uav["iha_enlem"]
            msg.pose.position.y = uav["iha_boylam"]
            msg.pose.position.z = uav["iha_irtifa"]
            msg.heading = float(uav["iha_yonelme"])
            msg.altitude = float(uav["iha_irtifa"])
            msg.hiz = float(uav["iha_hizi"])
            msg.zaman_farki = float(uav["zaman_farki"])

            msg.pose.orientation.w = 1.0
            msg.velocity.linear.x = msg.hiz

            target_pub.publish(msg)
        except KeyError as e:
            ros.logwarn(f"[Telemetri] Hatalı İHA bilgisi: {e} alanı eksik veya hatalı.")


def start_telemetry():
    ros.loginfo("[Telemetri] Telemetri paylaşımına başlanıyor...")
    ros.Timer(ros.Duration(1.0), send_telemetry)


target_pub = None
