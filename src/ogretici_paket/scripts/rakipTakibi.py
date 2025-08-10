#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

class FollowerController:
    """
    Bu sınıf, `GUIDED` modda ve `ARM` edilmiş bir takipçi İHA'yı kontrol eder.
    Hazırlık adımlarının (mod değiştirme, arm etme) kullanıcı tarafından yapıldığı varsayılır.
    Görevi: Liderin pozisyonunu dinleyerek takipçiye sürekli hedef noktası göndermek.
    """
    def __init__(self):
        # ROS düğümünü başlat
        rospy.init_node('pure_follower_node', anonymous=True)

        # --- Yapılandırma ---
        self.follower_ns = "mavros_rakip"
        self.leader_ns = "mavros"
        self.follow_distance = 50.0  # Takip mesafesi (metre)
        self.update_rate = 20.0      # Hedef gönderme frekansı (Hz)

        # --- Dahili Değişkenler ---
        self.leader_pose = None

        rospy.loginfo("Sadeleştirilmiş takip kontrolcüsü başlatıldı.")
        rospy.loginfo(f"'{self.leader_ns}' dinleniyor, '{self.follower_ns}' yönlendiriliyor.")

        # --- Subscriber ---
        # Liderin yerel pozisyonunu dinle
        rospy.Subscriber(
            f'/{self.leader_ns}/local_position/pose',
            PoseStamped,
            self.leader_pose_callback
        )

        # --- Publisher ---
        # Takipçiye hedef pozisyonu göndermek için
        self.follower_setpoint_pub = rospy.Publisher(
            f'/{self.follower_ns}/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

    def leader_pose_callback(self, msg):
        """Liderin pozisyonunu alır ve saklar."""
        self.leader_pose = msg

    def run(self):
        """Ana çalışma döngüsü."""
        rate = rospy.Rate(self.update_rate)

        # İlk pozisyon verisi gelene kadar bekle
        rospy.loginfo("Liderden ilk pozisyon verisinin gelmesi bekleniyor...")
        while not rospy.is_shutdown() and self.leader_pose is None:
            rate.sleep()
        
        rospy.loginfo("Lider pozisyonu alındı. Takip başlıyor.")

        while not rospy.is_shutdown():
            # leader_pose verisi mevcutsa devam et
            if self.leader_pose:
                # Yeni bir hedef pozisyon mesajı oluştur
                target_pose = PoseStamped()
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = "map"

                # Hedefi, liderin X ekseninde 50 metre gerisi olarak ayarla
                target_pose.pose.position.x = self.leader_pose.pose.position.x - self.follow_distance
                target_pose.pose.position.y = self.leader_pose.pose.position.y
                target_pose.pose.position.z = self.leader_pose.pose.position.z

                # Takipçinin yönelimini şimdilik lider ile aynı tutalım
                target_pose.pose.orientation = self.leader_pose.pose.orientation

                # Hesaplanan hedefi yayınla
                self.follower_setpoint_pub.publish(target_pose)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = FollowerController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Takip kontrolcüsü kapatıldı.")