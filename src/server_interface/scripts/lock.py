#!/usr/bin/env python3

import rospy as ros
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from server_interface.srv import LockOn, LockOnResponse

from scripts.auth_manager import AUTH_STATE
from scripts.common import post_json


def handle_lock_on(req):
    if not AUTH_STATE["logged_in"]:
        return LockOnResponse(False, "Not logged in")

    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/kilitlenme_bilgisi"

    payload = {
        "iha_id": AUTH_STATE["team_id"],
        "hedef_id": req.target_id,
        "merkez": {"x": req.center_x, "y": req.center_y},
        "genislik": req.width,
        "yukseklik": req.height,
    }

    required = ["iha_id", "hedef_id", "merkez", "genislik", "yukseklik"]
    if not all(k in payload for k in required):
        return LockOnResponse(False, "Missing required lock fields")

    status, response = post_json(url, payload)
    if status == 200:
        return LockOnResponse(True, "Lock-on reported successfully.")
    return LockOnResponse(False, f"Server error: status {status}")


if __name__ == "__main__":
    ros.init_node("lock_reporter")
    ros.Service("/lock_on_target", LockOn, handle_lock_on)
    ros.loginfo("[Lock] /lock_on_target servisi hazır.")
    ros.spin()
