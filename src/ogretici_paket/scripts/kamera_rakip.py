#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber:
    def __init__(self):
        # Düğümü (node) başlat
        rospy.init_node('camera_rakip_viewer', anonymous=True)
        rospy.loginfo("Kamera görüntüleyici düğümü başlatıldı.")

        # Görüntü dönüşümü için CvBridge nesnesi
        self.bridge = CvBridge()
        
        # Gelen görüntüyü saklamak için bir değişken
        self.cv_image = None
        
        # Abone olunacak ROS konusu (topic)
        self.image_topic = "/ardupilot_rakip_camera/image_raw"
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"'{self.image_topic}' konusuna abone olundu.")
        
    def image_callback(self, msg):
        """
        Bu fonksiyon her yeni görüntü mesajı geldiğinde ROS tarafından çağrılır.
        Görevi sadece gelen mesajı OpenCV formatına çevirip self.cv_image değişkenine kaydetmektir.
        """
        try:
            # Gelen ROS Image mesajını OpenCV formatına (BGR8) çevir.
            # 'bgr8' formatı cv2.imshow için en yaygın formattır.
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge hatası: {e}")

    def run(self):
        """
        Bu ana döngü, program çalıştığı sürece sürekli döner.
        Görüntüleri göstermekten ve kullanıcı girdisini kontrol etmekten sorumludur.
        """
        rospy.loginfo("Ana döngü başlatıldı. Görüntü bekleniyor...")
        
        # ROS kapatılmadığı sürece döngüye devam et
        while not rospy.is_shutdown():
            # Eğer image_callback tarafından yeni bir görüntü alındıysa
            if self.cv_image is not None:
                # Görüntüyü "Kamera Goruntusu" adlı pencerede göster
                cv2.imshow("Kamera Goruntusu", self.cv_image)
                
                # Pencerenin güncellenmesi ve klavye girdilerini işlemesi için
                # 1 milisaniye bekle. Bu satır siyah ekran sorununu çözer.
                # 'q' tuşuna basılıp basılmadığını kontrol et.
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("Kullanıcı 'q' tuşuna bastı. Program kapatılıyor.")
                    break # Döngüden çık

        # Döngü bittiğinde tüm OpenCV pencerelerini kapat
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV pencereleri kapatıldı.")

if __name__ == '__main__':
    try:
        # ImageSubscriber sınıfından bir nesne oluştur
        image_sub = ImageSubscriber()
        # Ana döngüyü başlat
        image_sub.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program ROS tarafından kesildi.")
    finally:
        # Her ihtimale karşı pencerelerin kapalı olduğundan emin ol
        cv2.destroyAllWindows()
