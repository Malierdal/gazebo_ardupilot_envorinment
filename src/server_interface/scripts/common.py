#!/usr/bin/env python3

import requests
import rospy as ros


def load_server_config():
    config = {}
    config["base_url"] = ros.get_param("/server/base_url")
    config["username"] = ros.get_param("/server/username")
    config["password"] = ros.get_param("/server/password")
    return config


def post_json(url, data):
    try:
        headers = {"Content-Type": "application/json"}
        response = requests.post(url, json=data, headers=headers, timeout=2.0)

        if response.status_code == 200:
            if response.content:
                return 200, response.json()
            else:
                return 200, {}
        elif response.status_code == 204:
            ros.logwarn(f"[POST] {url} 204 hata kodu döndürdü: İçerik yok veya hatalı.")
        elif response.status_code == 400:
            ros.logerr(f"[POST] {url} 400 hata kodu döndürdü: İstek biçimi hatalı.")
        elif response.status_code == 401:
            ros.logerr(f"[POST] {url} 401 hata kodu döndürdü: Yetkisiz istek.")
        elif response.status_code == 403:
            ros.logerr(f"[POST] {url} 403 hata kodu döndürdü: Yasaklı.")
        elif response.status_code == 404:
            ros.logerr(f"[POST] {url} 404 hata kodu döndürdü: Adres bulunamadı.")
        elif response.status_code == 500:
            ros.logerr(f"[POST] {url} 500 hata kodu döndürdü: Sunucu hatası.")
        else:
            ros.logwarn(f"[POST] {url} beklenmeyen hata kodu: {response.status_code}")

        return response.status_code, {}
    except requests.exceptions.RequestException as e:
        ros.logerr(f"[POST] {url} adresine istek başarısız oldu: {e}")
        return None, {}


def get_json(url):
    try:
        response = requests.get(url, timeout=2.0)

        if response.status_code == 200:
            if response.content:
                return 200, response.json()
            else:
                return 200, {}
        elif response.status_code == 204:
            ros.logwarn(f"[GET] {url} 204 hata kodu döndürdü: İçerik yok veya hatalı.")
        elif response.status_code == 400:
            ros.logerr(f"[GET] {url} 400 hata kodu döndürdü: İstek biçimi hatalı.")
        elif response.status_code == 401:
            ros.logerr(f"[GET] {url} 401 hata kodu döndürdü: Yetkisiz istek.")
        elif response.status_code == 403:
            ros.logerr(f"[GET] {url} 403 hata kodu döndürdü: Yasaklı.")
        elif response.status_code == 404:
            ros.logerr(f"[GET] {url} 404 hata kodu döndürdü: Adres bulunamadı.")
        elif response.status_code == 500:
            ros.logerr(f"[GET] {url} 500 hata kodu döndürdü: Sunucu hatası.")
        else:
            ros.logwarn(f"[GET] {url} unexpected status: {response.status_code}")

        return response.status_code, response.json() if response.content else {}
    except requests.exceptions.RequestException as e:
        ros.logwarn(f"[GET] {url} adresine istek başarısız oldu: {e}")
        return None, {}


def validate_payload(payload, required_fields):
    missing = [key for key in required_fields if key not in payload]
    if missing:
        ros.logerr(f"Bazı alanlar eksik gönderildi: {missing}")
        return False
    return True
