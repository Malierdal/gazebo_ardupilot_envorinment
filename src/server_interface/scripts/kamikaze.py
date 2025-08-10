#!/usr/bin/env python3

import rospy as ros
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from server_interface.srv import SubmitKamikaze, SubmitKamikazeResponse

from scripts.auth_manager import AUTH_STATE
from scripts.common import post_json


def handle_kamikaze(req):
    if not AUTH_STATE["logged_in"]:
        return SubmitKamikazeResponse(False, "Not logged in.")

    base_url = ros.get_param("/server/base_url")
    url = f"{base_url}/api/kamikaze_bilgisi"

    payload = {"iha_id": AUTH_STATE["team_id"], "metin": req.text}

    status, response = post_json(url, payload)
    if status == 200:
        ros.loginfo("[Kamikaze] QR text submitted.")
        return SubmitKamikazeResponse(True, "QR text submitted.")
    else:
        return SubmitKamikazeResponse(False, f"Error {status}")


if __name__ == "__main__":
    ros.init_node("kamikaze_submitter")
    ros.Service("/submit_kamikaze", SubmitKamikaze, handle_kamikaze)
    ros.loginfo("[Kamikaze] /submit_kamikaze servisi hazır.")
    ros.spin()
