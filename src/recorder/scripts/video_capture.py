#!/usr/bin/env python3

import os
import subprocess
import time
from datetime import datetime
from typing import Dict, cast

import rospy as ros


def load_camera_config():
    camera_mode = cast(str, ros.get_param("/default_camera_mode", "hd_standard"))
    all_modes = cast(Dict[str, Dict[str, int]], ros.get_param("/camera_modes", {}))

    if camera_mode not in all_modes:
        ros.logerr(f"{camera_mode} kamera modu geçerli modlar arasında bulunamadı.")
        ros.signal_shutdown("[Video] Geçersiz kamera modu belirtildi.")
        return {}

    selected = all_modes[camera_mode]
    ros.loginfo(f"[Video] Kamera modu seçildi: {camera_mode}")
    return {
        "width": selected.get("width", 640),
        "height": selected.get("height", 480),
        "framerate": selected.get("framerate", 30),
    }


def wait_for_topic(topic_name, timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            published_topics = ros.get_published_topics()
            if any(topic == topic_name for topic, _ in published_topics):
                ros.loginfo(f"[Video] Topic aktif: {topic_name}")
                return True
        except ros.ROSException:
            pass
        time.sleep(0.5)
    ros.logwarn(f"[Video] {topic_name} beklenirken zaman aşımı.")
    return False


def create_session_directory(base_path="~/catkin_ws/flight_bags"):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(os.path.expanduser(base_path), timestamp)
    os.makedirs(session_dir, exist_ok=True)
    return session_dir


def start_usb_cam_node(video_device="/dev/video0", width=1920, height=1080):
    cmd = [
        "rosrun",
        "usb_cam",
        "usb_cam_node",
        f"_video_device:={video_device}",
        f"_image_width:={width}",
        f"_image_height:={height}",
        "_pixel_format:=mjpeg",
        "_camera_frame_id:=usb_cam",
        "_io_method:=mmap",
    ]
    try:
        ros.loginfo(f"[USB_CAM] Başlatılıyor: {' '.join(cmd)}")
        return subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    except FileNotFoundError:
        ros.logerr("[Video] usb_cam_node bulunamadı.")
        ros.signal_shutdown("[Video] usb_cam_node başlatılamadı.")
    except Exception as e:
        ros.logerr(f"[Video] usb_cam_node hatası: {e}")
        ros.signal_shutdown("[Video] usb_cam başlatılamadı.")
    return None


def start_rosbag_recording(output_dir: str):
    bag_filename = os.path.join(output_dir, "flight.bag")
    topics = [
        "/mavros/state",
        "/mavros/local_position/pose",
        "/mavros/global_position/global",
        "/mavros/global_position/raw/fix",
        "/mavros/global_position/compass_hdg",
        "/mavros/imu/data",
        "/mavros/battery",
        "/usb_cam/camera_info",
        "/yolo_detections",
        "/tracker/target_pose",
        "/referee_data",
        "/flight_mode",
        "/pid_debug",
        "/rosout",
        "/rosout_agg",
        "/usb_cam/image_raw/compressed",
        "/usb_cam/image_raw/compressed/parameter_updates",
        "/usb_cam/image_raw/compressed/parameter_descriptions",
    ]
    cmd = ["rosbag", "record", "-O", bag_filename] + topics
    ros.loginfo(f"[Video] rosbag kaydı başlatılıyor: {bag_filename}")
    return subprocess.Popen(cmd)


def main():
    ros.init_node("video_capture", anonymous=False)
    ros.loginfo("[Video] Video kaydı başlatıldı.")

    config = load_camera_config()
    width = config.get("width", 1920)
    height = config.get("height", 1080)

    ros.loginfo(f"[Video] Çözünürlük: {width}x{height}")

    session_dir = create_session_directory()
    ros.loginfo(f"[Video] Kayıt klasörü: {session_dir}")

    usb_cam_proc = start_usb_cam_node("/dev/video0", width, height)

    required_topics = [
        "/usb_cam/image_raw",
        "/mavros/state",
        "/mavros/imu/data",
        "/mavros/local_position/pose",
    ]

    for topic in required_topics:
        wait_for_topic(topic, timeout=10)

    rosbag_proc = start_rosbag_recording(session_dir)

    try:
        ros.spin()
    except ros.ROSInterruptException:
        ros.loginfo("[Video] Kayıt manuel olarak durduruldu.")
    finally:
        if usb_cam_proc and usb_cam_proc.poll() is None:
            usb_cam_proc.terminate()
            ros.loginfo("[Video] usb_cam_node durduruldu.")
        if rosbag_proc and rosbag_proc.poll() is None:
            rosbag_proc.terminate()
            ros.loginfo("[Video] rosbag kaydı durduruldu.")


if __name__ == "__main__":
    main()
