#!/usr/bin/env python3

import rospy as ros
from std_srvs.srv import Trigger, TriggerResponse

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from scripts.common import load_server_config, post_json, validate_payload

AUTH_STATE = {"logged_in": False, "team_id": None}


def login():
    config = load_server_config()
    url = f"{config['base_url']}/api/giris"
    payload = {
        "kadi": config["username"],
        "sifre": config["password"],
        # "takimsifresi" ?
    }

    required_fields = ["kadi", "sifre"]
    if not validate_payload(payload, required_fields):
        return False

    status, response = post_json(url, payload)

    if status == 200:
        AUTH_STATE["logged_in"] = True
        AUTH_STATE["team_id"] = response
        ros.loginfo(f"[Giriş] Başarılı. Takım ID'si: {response}")
        return True
    elif status == 400:
        ros.logerr("[Giriş] Başarısız: Geçersiz kullanıcı adı veya şifre.")
    else:
        ros.logwarn(f"[Giriş] Beklenmeyen hata kodu: {status}")
    return False


def handle_force_login(req):
    ros.loginfo("[Giriş] Manuel giriş talep edildi...")
    success = login()
    return TriggerResponse(
        success=success, message="Giriş başarılı" if success else "Giriş başarısız"
    )


def init_auth_service():
    ros.Service("/force_login", Trigger, handle_force_login)
    ros.loginfo("[Giriş] /force_login servisi hazır.")
