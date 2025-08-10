#! /usr/bin/env python3

import rospy as ros
from std_msgs.msg import Header
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from scripts.common import get_json

server_time_pub = None


def query_server_time():
    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/sunucusaati"
    status, response = get_json(url)

    if status != 200:
        ros.logwarn_throttle(
            5.0, f"[Zaman Senkronizasyonu] Sunucu hata kodu döndürdü: {status}"
        )
        return None

    if not all(
        k in response for k in ["zaman", "saat", "dakika", "saniye", "milisaniye"]
    ):
        ros.logwarn("[Zaman Senkronizasyonu] Sunucu yanıtında eksik alanlar var.")
        return None

    ros.loginfo("[Zaman Senkronizasyonu] Sunucu zamanı alındı: %s", response["zaman"])

    if server_time_pub is not None:
        stamp = ros.Time.now()
        msg = Header()
        msg.stamp = stamp
        msg.frame_id = f"Server Time: {response['saat']}:{response['dakika']}:{response['saniye']}.{response['milisaniye']}"
        server_time_pub.publish(msg)


def start_time_sync(interval_sec=10.0):
    global server_time_pub
    ros.loginfo(
        f"[Zaman Senkronizasyonu] Sunucu zamanı her {interval_sec} saniyede bir sorgulanacak. Sorgulama başlatılıyor."
    )
    server_time_pub = ros.Publisher("/server_time", Header, queue_size=5)
    ros.Timer(ros.Duration(interval_sec), query_server_time)
