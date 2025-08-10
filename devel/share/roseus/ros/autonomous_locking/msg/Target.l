;; Auto-generated. Do not edit!


(when (boundp 'autonomous_locking::Target)
  (if (not (find-package "AUTONOMOUS_LOCKING"))
    (make-package "AUTONOMOUS_LOCKING"))
  (shadow 'Target (find-package "AUTONOMOUS_LOCKING")))
(unless (find-package "AUTONOMOUS_LOCKING::TARGET")
  (make-package "AUTONOMOUS_LOCKING::TARGET"))

(in-package "ROS")
;;//! \htmlinclude Target.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass autonomous_locking::Target
  :super ros::object
  :slots (_id _pose _velocity _heading _speed _time_delay _distance _tail_angle _heading_alignment _zone_score _total_cost ))

(defmethod autonomous_locking::Target
  (:init
   (&key
    ((:id __id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:velocity __velocity) (instance geometry_msgs::Twist :init))
    ((:heading __heading) 0.0)
    ((:speed __speed) 0.0)
    ((:time_delay __time_delay) 0.0)
    ((:distance __distance) 0.0)
    ((:tail_angle __tail_angle) 0.0)
    ((:heading_alignment __heading_alignment) 0.0)
    ((:zone_score __zone_score) 0.0)
    ((:total_cost __total_cost) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pose __pose)
   (setq _velocity __velocity)
   (setq _heading (float __heading))
   (setq _speed (float __speed))
   (setq _time_delay (float __time_delay))
   (setq _distance (float __distance))
   (setq _tail_angle (float __tail_angle))
   (setq _heading_alignment (float __heading_alignment))
   (setq _zone_score (float __zone_score))
   (setq _total_cost (float __total_cost))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:time_delay
   (&optional __time_delay)
   (if __time_delay (setq _time_delay __time_delay)) _time_delay)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:tail_angle
   (&optional __tail_angle)
   (if __tail_angle (setq _tail_angle __tail_angle)) _tail_angle)
  (:heading_alignment
   (&optional __heading_alignment)
   (if __heading_alignment (setq _heading_alignment __heading_alignment)) _heading_alignment)
  (:zone_score
   (&optional __zone_score)
   (if __zone_score (setq _zone_score __zone_score)) _zone_score)
  (:total_cost
   (&optional __total_cost)
   (if __total_cost (setq _total_cost __total_cost)) _total_cost)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _velocity
    (send _velocity :serialization-length)
    ;; float32 _heading
    4
    ;; float32 _speed
    4
    ;; float32 _time_delay
    4
    ;; float32 _distance
    4
    ;; float32 _tail_angle
    4
    ;; float32 _heading_alignment
    4
    ;; float32 _zone_score
    4
    ;; float32 _total_cost
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _velocity
       (send _velocity :serialize s)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_delay
       (sys::poke _time_delay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tail_angle
       (sys::poke _tail_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_alignment
       (sys::poke _heading_alignment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zone_score
       (sys::poke _zone_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_cost
       (sys::poke _total_cost (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_delay
     (setq _time_delay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tail_angle
     (setq _tail_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_alignment
     (setq _heading_alignment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zone_score
     (setq _zone_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_cost
     (setq _total_cost (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get autonomous_locking::Target :md5sum-) "563c719c20ae1eacdb952c207c883d48")
(setf (get autonomous_locking::Target :datatype-) "autonomous_locking/Target")
(setf (get autonomous_locking::Target :definition-)
      "int32 id
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
float32 heading
float32 speed
float32 time_delay

float32 distance
float32 tail_angle
float32 heading_alignment
float32 zone_score
float32 total_cost

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :autonomous_locking/Target "563c719c20ae1eacdb952c207c883d48")


