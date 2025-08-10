; Auto-generated. Do not edit!


(cl:in-package server_interface-srv)


;//! \htmlinclude LockOn-request.msg.html

(cl:defclass <LockOn-request> (roslisp-msg-protocol:ros-message)
  ((target_id
    :reader target_id
    :initarg :target_id
    :type cl:integer
    :initform 0)
   (center_x
    :reader center_x
    :initarg :center_x
    :type cl:integer
    :initform 0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0))
)

(cl:defclass LockOn-request (<LockOn-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LockOn-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LockOn-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_interface-srv:<LockOn-request> is deprecated: use server_interface-srv:LockOn-request instead.")))

(cl:ensure-generic-function 'target_id-val :lambda-list '(m))
(cl:defmethod target_id-val ((m <LockOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:target_id-val is deprecated.  Use server_interface-srv:target_id instead.")
  (target_id m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <LockOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:center_x-val is deprecated.  Use server_interface-srv:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <LockOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:center_y-val is deprecated.  Use server_interface-srv:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <LockOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:width-val is deprecated.  Use server_interface-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <LockOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:height-val is deprecated.  Use server_interface-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LockOn-request>) ostream)
  "Serializes a message object of type '<LockOn-request>"
  (cl:let* ((signed (cl:slot-value msg 'target_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'center_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'center_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LockOn-request>) istream)
  "Deserializes a message object of type '<LockOn-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LockOn-request>)))
  "Returns string type for a service object of type '<LockOn-request>"
  "server_interface/LockOnRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockOn-request)))
  "Returns string type for a service object of type 'LockOn-request"
  "server_interface/LockOnRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LockOn-request>)))
  "Returns md5sum for a message object of type '<LockOn-request>"
  "685ca96bd6cc7d98e3174f48c9825cf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LockOn-request)))
  "Returns md5sum for a message object of type 'LockOn-request"
  "685ca96bd6cc7d98e3174f48c9825cf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LockOn-request>)))
  "Returns full string definition for message of type '<LockOn-request>"
  (cl:format cl:nil "int32 target_id~%int32 center_x~%int32 center_y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LockOn-request)))
  "Returns full string definition for message of type 'LockOn-request"
  (cl:format cl:nil "int32 target_id~%int32 center_x~%int32 center_y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LockOn-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LockOn-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LockOn-request
    (cl:cons ':target_id (target_id msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
;//! \htmlinclude LockOn-response.msg.html

(cl:defclass <LockOn-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass LockOn-response (<LockOn-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LockOn-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LockOn-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_interface-srv:<LockOn-response> is deprecated: use server_interface-srv:LockOn-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LockOn-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:success-val is deprecated.  Use server_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <LockOn-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_interface-srv:message-val is deprecated.  Use server_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LockOn-response>) ostream)
  "Serializes a message object of type '<LockOn-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LockOn-response>) istream)
  "Deserializes a message object of type '<LockOn-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LockOn-response>)))
  "Returns string type for a service object of type '<LockOn-response>"
  "server_interface/LockOnResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockOn-response)))
  "Returns string type for a service object of type 'LockOn-response"
  "server_interface/LockOnResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LockOn-response>)))
  "Returns md5sum for a message object of type '<LockOn-response>"
  "685ca96bd6cc7d98e3174f48c9825cf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LockOn-response)))
  "Returns md5sum for a message object of type 'LockOn-response"
  "685ca96bd6cc7d98e3174f48c9825cf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LockOn-response>)))
  "Returns full string definition for message of type '<LockOn-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LockOn-response)))
  "Returns full string definition for message of type 'LockOn-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LockOn-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LockOn-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LockOn-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LockOn)))
  'LockOn-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LockOn)))
  'LockOn-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockOn)))
  "Returns string type for a service object of type '<LockOn>"
  "server_interface/LockOn")