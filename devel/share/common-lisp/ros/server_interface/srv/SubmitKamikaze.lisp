; Auto-generated. Do not edit!


(cl:in-package server_interface-srv)


;//! \htmlinclude SubmitKamikaze-request.msg.html

(cl:defclass <SubmitKamikaze-request> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass SubmitKamikaze-request (<SubmitKamikaze-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubmitKamikaze-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubmitKamikaze-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_interface-srv:<SubmitKamikaze-request> is deprecated: use server_interface-srv:SubmitKamikaze-request instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <SubmitKamikaze-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:text-val is deprecated.  Use server_interface-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubmitKamikaze-request>) ostream)
  "Serializes a message object of type '<SubmitKamikaze-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubmitKamikaze-request>) istream)
  "Deserializes a message object of type '<SubmitKamikaze-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubmitKamikaze-request>)))
  "Returns string type for a service object of type '<SubmitKamikaze-request>"
  "server_interface/SubmitKamikazeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitKamikaze-request)))
  "Returns string type for a service object of type 'SubmitKamikaze-request"
  "server_interface/SubmitKamikazeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubmitKamikaze-request>)))
  "Returns md5sum for a message object of type '<SubmitKamikaze-request>"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubmitKamikaze-request)))
  "Returns md5sum for a message object of type 'SubmitKamikaze-request"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubmitKamikaze-request>)))
  "Returns full string definition for message of type '<SubmitKamikaze-request>"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubmitKamikaze-request)))
  "Returns full string definition for message of type 'SubmitKamikaze-request"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubmitKamikaze-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubmitKamikaze-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SubmitKamikaze-request
    (cl:cons ':text (text msg))
))
;//! \htmlinclude SubmitKamikaze-response.msg.html

(cl:defclass <SubmitKamikaze-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SubmitKamikaze-response (<SubmitKamikaze-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubmitKamikaze-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubmitKamikaze-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_interface-srv:<SubmitKamikaze-response> is deprecated: use server_interface-srv:SubmitKamikaze-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SubmitKamikaze-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:success-val is deprecated.  Use server_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SubmitKamikaze-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:message-val is deprecated.  Use server_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubmitKamikaze-response>) ostream)
  "Serializes a message object of type '<SubmitKamikaze-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubmitKamikaze-response>) istream)
  "Deserializes a message object of type '<SubmitKamikaze-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubmitKamikaze-response>)))
  "Returns string type for a service object of type '<SubmitKamikaze-response>"
  "server_interface/SubmitKamikazeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitKamikaze-response)))
  "Returns string type for a service object of type 'SubmitKamikaze-response"
  "server_interface/SubmitKamikazeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubmitKamikaze-response>)))
  "Returns md5sum for a message object of type '<SubmitKamikaze-response>"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubmitKamikaze-response)))
  "Returns md5sum for a message object of type 'SubmitKamikaze-response"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubmitKamikaze-response>)))
  "Returns full string definition for message of type '<SubmitKamikaze-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubmitKamikaze-response)))
  "Returns full string definition for message of type 'SubmitKamikaze-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubmitKamikaze-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubmitKamikaze-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SubmitKamikaze-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SubmitKamikaze)))
  'SubmitKamikaze-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SubmitKamikaze)))
  'SubmitKamikaze-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitKamikaze)))
  "Returns string type for a service object of type '<SubmitKamikaze>"
  "server_interface/SubmitKamikaze")