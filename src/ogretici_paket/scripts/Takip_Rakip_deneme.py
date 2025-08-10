#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class ChaserPlaneController:
    """
    Bu sınıf, başka bir uçağın (hedef) konumunu dinleyerek onu belirli bir
    mesafe ve yükseklik farkıyla takip eden takipçi uçağı kontrol eder.
    """
    def __init__(self, chaser_ns, target_ns):
        self.current_state = None
        self.target_pose = None # Bu, HEDEF uçağın pozisyonudur.

        rospy.loginfo(f"Takipçi Uçak Kontrolcüsü başlatılıyor. Takipçi: '{chaser_ns}', Hedef: '{target_ns}'")

        # --- Subscriber'lar ---
        # 1. Kendi durumumuzu dinle (Takipçi Uçak)
        rospy.Subscriber(f"/{chaser_ns}/mavros/state", State, self.state_cb)
        
        # 2. HEDEF uçağın pozisyonunu dinle
        rospy.Subscriber(f"/{target_ns}/mavros_rakip/local_position/pose", PoseStamped, self.target_pose_cb)

        # --- Publisher ---
        # 1. Kendi hedef noktamızı yayınla (Takipçi Uçak)
        self.local_pos_pub = rospy.Publisher(f"/{chaser_ns}/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # --- Servis İstemcileri ---
        rospy.wait_for_service(f'/{chaser_ns}/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy(f'/{chaser_ns}/mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service(f'/{chaser_ns}/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy(f'/{chaser_ns}/mavros/set_mode', SetMode)
        
        rospy.loginfo("Kontrolcü başlatıldı.")

    def state_cb(self, msg):
        self.current_state = msg

    def target_pose_cb(self, msg):
        # Hedef uçağın pozisyonu güncellendiğinde bu fonksiyon çalışır.
        self.target_pose = msg
    
    def set_mode_and_arm(self):
        """Takipçi uçağı GUIDED moda alıp arm eder."""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.current_state is None:
            rospy.loginfo_throttle(5, "Takipçi uçağın MAVROS durumu bekleniyor...")
            rate.sleep()

        if self.current_state.mode != "GUIDED":
            if self.set_mode_client(custom_mode="GUIDED").mode_sent:
                rospy.loginfo("Takipçi uçak GUIDED moda alındı.")
            else:
                rospy.logerr("GUIDED moda geçilemedi."); return False

        if not self.current_state.armed:
            if self.arming_client(value=True).success:
                rospy.loginfo("Takipçi uçak ARM edildi.")
            else:
                rospy.logerr("Arm etme başarısız."); return False
        
        return True

    def follow_target(self, follow_distance_m, altitude_offset_m, rate):
        """
        Hedefin pozisyonunu alıp, belirlenen mesafe ve yükseklik farkıyla
        sürekli olarak yeni hedef noktası yayınlar.
        """
        rospy.loginfo("Hedef takip döngüsü başlıyor...")
        while not rospy.is_shutdown():
            if self.target_pose is None:
                rospy.logwarn_throttle(10, "Hedef uçağın pozisyonu bekleniyor...")
                rate.sleep()
                continue

            # Hedefin pozisyonuna göre kendi hedefimizi hesapla
            chaser_target_pose = PoseStamped()
            chaser_target_pose.header.stamp = rospy.Time.now()
            chaser_target_pose.header.frame_id = "map"

            # Basit bir takip mantığı: Hedefin tam arkasında dur
            # (Daha karmaşık takip mantıkları eklenebilir)
            chaser_target_pose.pose.position.x = self.target_pose.pose.position.x - follow_distance_m
            chaser_target_pose.pose.position.y = self.target_pose.pose.position.y
            chaser_target_pose.pose.position.z = self.target_pose.pose.position.z + altitude_offset_m
            
            # Kendi hedef noktamızı yayınla
            self.local_pos_pub.publish(chaser_target_pose)
            
            rospy.loginfo_throttle(5, f"Hedefin pozisyonu: [X:{self.target_pose.pose.position.x:.1f}, Y:{self.target_pose.pose.position.y:.1f}, Z:{self.target_pose.pose.position.z:.1f}] -> Takipçinin hedefi hesaplandı.")
            rate.sleep()

# --- ANA PROGRAM BLOĞU ---
if __name__ == "__main__":
    rospy.init_node("chaser_plane_node")
    
    # --- İsim Alanları ve Görev Parametreleri ---
    chaser_namespace = "plane1"  # Takipçi uçağın isim alanı
    target_namespace = "plane2"  # Hedef uçağın isim alanı
    
    follow_distance = 100.0      # Hedefin ne kadar arkasından takip edeceği (metre)
    altitude_offset = 20.0       # Hedefin ne kadar yukarısından takip edeceği (metre)

    try:
        controller = ChaserPlaneController(chaser_namespace, target_namespace)
        rate = rospy.Rate(10) # 10 Hz

        # Adım 1: Takipçi uçağı uçuşa hazırla
        if not controller.set_mode_and_arm():
            rospy.signal_shutdown("Uçuşa hazırlık başarısız, program sonlandırılıyor.")

        rospy.loginfo("Takipçi uçak göreve hazır. Hedef bekleniyor...")
        rospy.sleep(2)

        # Adım 2: Takip döngüsünü başlat
        controller.follow_target(follow_distance, altitude_offset, rate)

    except rospy.ROSInterruptException:
        rospy.logerr("Program kesildi.")
    finally:
        rospy.loginfo("Program sonlandırılıyor.")