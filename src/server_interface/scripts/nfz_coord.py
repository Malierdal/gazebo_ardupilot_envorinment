#!/usr/bin/env python3

import os
import sys
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rospy as ros
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import Trigger, TriggerResponse

from scripts.common import get_json

nfz_pub: Optional[ros.Publisher] = None


def query_nfz_coords():
    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/nfz_koordinatlari"

    status, response = get_json(url)

    if status != 200:
        ros.logwarn(f"[HSS] HSS verisi alınamadı. Hata kodu: {status}")
        return []

    if "nfz_koordinat_bilgileri" not in response:
        ros.loginfo("[HSS] Aktif HSS bölgeleri bulunamadı.")
        return []

    zones = []
    for entry in response["nfz_koordinat_bilgileri"]:
        try:
            p = Point()
            p.x = float(entry["hssEnlem"])
            p.y = float(entry["hssBoylam"])
            p.z = float(entry["hssYaricap"])  # z = yarıçap
            zones.append(p)
        except KeyError:
            ros.logwarn(f"[HSS] Hatalı HSS girdisi atlanıyor: {entry}")
    return zones


def handle_nfz_service(req):
    zones = query_nfz_coords()
    if not zones:
        return TriggerResponse(success=False, message="HSS verisi alınamadı.")

    for i, zone in enumerate(zones):
        msg = PointStamped()
        msg.header.stamp = ros.Time.now()
        msg.header.frame_id = f"NFZ_{i}"
        msg.point = zone
        if nfz_pub is not None:
            nfz_pub.publish(msg)
        else:
            ros.logwarn("nfz_pub not initialized.")

    return TriggerResponse(
        success=True, message=f"{len(zones)} HSS bölgesi yayınlandı."
    )


if __name__ == "__main__":
    ros.init_node("nfz_coord")
    nfz_pub = ros.Publisher("/nfz_coords", PointStamped, queue_size=10, latch=True)
    ros.Service("/get_nfz_coords", Trigger, handle_nfz_service)
    ros.loginfo("[HSS] /get_nfz_coords servisi hazır.")
    ros.spin()
