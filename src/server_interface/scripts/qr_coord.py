#!/usr/bin/env python3

from typing import Optional
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rospy as ros
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse

from scripts.common import get_json

qr_pub: Optional[ros.Publisher] = None


def query_qr_coords():
    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/qr_koordinati"
    status, response = get_json(url)

    if status != 200:
        ros.logwarn_throttle(5.0, f"[QR] Sunucu hata kodu döndürdü: {status}")
        return None

    try:
        lat = float(response["qrEnlem"])
        lon = float(response["qrBoylam"])
        ros.loginfo(f"[QR] Koordinatlar alındı: lat={lat} lon={lon}")
        return lat, lon
    except KeyError:
        ros.logwarn("[QR] QR koordinatları alınırken bir hata oluştu.")
        return None


def handle_qr_service(req):
    coords = query_qr_coords()
    if coords is not None:
        lat, lon = coords
        msg = Point(x=lat, y=lon, z=0)
        if qr_pub is not None:
            qr_pub.publish(msg)
        else:
            ros.logwarn("qr_pub not initialized.")
        return TriggerResponse(success=True, message="QR koordinatları yayınlandı.")
    return TriggerResponse(success=False, message="Koordinatlar alınamadı.")


if __name__ == "__main__":
    ros.init_node("qr_coord")
    qr_pub = ros.Publisher("/qr_coords", Point, queue_size=1, latch=True)
    ros.Service("/get_qr_coords", Trigger, handle_qr_service)
    ros.loginfo("[QR] /get_qr_coords servisi hazır.")
    ros.spin()
