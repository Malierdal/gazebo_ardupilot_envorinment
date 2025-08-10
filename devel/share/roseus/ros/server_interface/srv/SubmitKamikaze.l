;; Auto-generated. Do not edit!


(when (boundp 'server_interface::SubmitKamikaze)
  (if (not (find-package "SERVER_INTERFACE"))
    (make-package "SERVER_INTERFACE"))
  (shadow 'SubmitKamikaze (find-package "SERVER_INTERFACE")))
(unless (find-package "SERVER_INTERFACE::SUBMITKAMIKAZE")
  (make-package "SERVER_INTERFACE::SUBMITKAMIKAZE"))
(unless (find-package "SERVER_INTERFACE::SUBMITKAMIKAZEREQUEST")
  (make-package "SERVER_INTERFACE::SUBMITKAMIKAZEREQUEST"))
(unless (find-package "SERVER_INTERFACE::SUBMITKAMIKAZERESPONSE")
  (make-package "SERVER_INTERFACE::SUBMITKAMIKAZERESPONSE"))

(in-package "ROS")





(defclass server_interface::SubmitKamikazeRequest
  :super ros::object
  :slots (_text ))

(defmethod server_interface::SubmitKamikazeRequest
  (:init
   (&key
    ((:text __text) "")
    )
   (send-super :init)
   (setq _text (string __text))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass server_interface::SubmitKamikazeResponse
  :super ros::object
  :slots (_success _message ))

(defmethod server_interface::SubmitKamikazeResponse
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

(defclass server_interface::SubmitKamikaze
  :super ros::object
  :slots ())

(setf (get server_interface::SubmitKamikaze :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get server_interface::SubmitKamikaze :datatype-) "server_interface/SubmitKamikaze")
(setf (get server_interface::SubmitKamikaze :request) server_interface::SubmitKamikazeRequest)
(setf (get server_interface::SubmitKamikaze :response) server_interface::SubmitKamikazeResponse)

(defmethod server_interface::SubmitKamikazeRequest
  (:response () (instance server_interface::SubmitKamikazeResponse :init)))

(setf (get server_interface::SubmitKamikazeRequest :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get server_interface::SubmitKamikazeRequest :datatype-) "server_interface/SubmitKamikazeRequest")
(setf (get server_interface::SubmitKamikazeRequest :definition-)
      "string text
---
bool success
string message
")

(setf (get server_interface::SubmitKamikazeResponse :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get server_interface::SubmitKamikazeResponse :datatype-) "server_interface/SubmitKamikazeResponse")
(setf (get server_interface::SubmitKamikazeResponse :definition-)
      "string text
---
bool success
string message
")



(provide :server_interface/SubmitKamikaze "191f0a61ba3a929ce6cac14343f74651")


