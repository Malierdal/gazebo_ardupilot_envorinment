#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# YOLO modelini yükle
model = YOLO("yolov8_100ep.pt")
bridge = CvBridge()

def image_callback(msg):
    try:
        # ROS görüntüsünü OpenCV formatına çevir
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"Görüntü dönüşüm hatası: {e}")
        return

    # YOLO ile tahmin yap
    results = model.predict(source=frame, conf=0.3, show=False)

    # Sonucu çiz
    annotated = results[0].plot()

    # OpenCV ile göster
    cv2.imshow("YOLOv8 - Simülasyon Kamerası", annotated)
    if cv2.waitKey(1) == ord('q'):
        rospy.signal_shutdown("Q ile çıkıldı.")

def main():
    rospy.init_node("yolo_sim_camera_node")
    rospy.Subscriber("/ardupilot_camera/image_raw", Image, image_callback)
    rospy.loginfo("YOLOv8, simülasyon kamerasını dinliyor...")

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
