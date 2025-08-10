;; Auto-generated. Do not edit!


(when (boundp 'server_interface::LockOn)
  (if (not (find-package "SERVER_INTERFACE"))
    (make-package "SERVER_INTERFACE"))
  (shadow 'LockOn (find-package "SERVER_INTERFACE")))
(unless (find-package "SERVER_INTERFACE::LOCKON")
  (make-package "SERVER_INTERFACE::LOCKON"))
(unless (find-package "SERVER_INTERFACE::LOCKONREQUEST")
  (make-package "SERVER_INTERFACE::LOCKONREQUEST"))
(unless (find-package "SERVER_INTERFACE::LOCKONRESPONSE")
  (make-package "SERVER_INTERFACE::LOCKONRESPONSE"))

(in-package "ROS")





(defclass server_interface::LockOnRequest
  :super ros::object
  :slots (_target_id _center_x _center_y _width _height ))

(defmethod server_interface::LockOnRequest
  (:init
   (&key
    ((:target_id __target_id) 0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _target_id (round __target_id))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:target_id
   (&optional __target_id)
   (if __target_id (setq _target_id __target_id)) _target_id)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; int32 _target_id
    4
    ;; int32 _center_x
    4
    ;; int32 _center_y
    4
    ;; int32 _width
    4
    ;; int32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target_id
       (write-long _target_id s)
     ;; int32 _center_x
       (write-long _center_x s)
     ;; int32 _center_y
       (write-long _center_y s)
     ;; int32 _width
       (write-long _width s)
     ;; int32 _height
       (write-long _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target_id
     (setq _target_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_x
     (setq _center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_y
     (setq _center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass server_interface::LockOnResponse
  :super ros::object
  :slots (_success _message ))

(defmethod server_interface::LockOnResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass server_interface::LockOn
  :super ros::object
  :slots ())

(setf (get server_interface::LockOn :md5sum-) "685ca96bd6cc7d98e3174f48c9825cf9")
(setf (get server_interface::LockOn :datatype-) "server_interface/LockOn")
(setf (get server_interface::LockOn :request) server_interface::LockOnRequest)
(setf (get server_interface::LockOn :response) server_interface::LockOnResponse)

(defmethod server_interface::LockOnRequest
  (:response () (instance server_interface::LockOnResponse :init)))

(setf (get server_interface::LockOnRequest :md5sum-) "685ca96bd6cc7d98e3174f48c9825cf9")
(setf (get server_interface::LockOnRequest :datatype-) "server_interface/LockOnRequest")
(setf (get server_interface::LockOnRequest :definition-)
      "int32 target_id
int32 center_x
int32 center_y
int32 width
int32 height
---
bool success
string message
")

(setf (get server_interface::LockOnResponse :md5sum-) "685ca96bd6cc7d98e3174f48c9825cf9")
(setf (get server_interface::LockOnResponse :datatype-) "server_interface/LockOnResponse")
(setf (get server_interface::LockOnResponse :definition-)
      "int32 target_id
int32 center_x
int32 center_y
int32 width
int32 height
---
bool success
string message
")



(provide :server_interface/LockOn "685ca96bd6cc7d98e3174f48c9825cf9")


