; Auto-generated. Do not edit!


(cl:in-package sharps-msg)


;//! \htmlinclude Distance.msg.html

(cl:defclass <Distance> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rear
    :reader rear
    :initarg :rear
    :type cl:float
    :initform 0.0)
   (front
    :reader front
    :initarg :front
    :type cl:float
    :initform 0.0)
   (rear_r
    :reader rear_r
    :initarg :rear_r
    :type cl:float
    :initform 0.0)
   (front_r
    :reader front_r
    :initarg :front_r
    :type cl:float
    :initform 0.0)
   (rear_l
    :reader rear_l
    :initarg :rear_l
    :type cl:float
    :initform 0.0)
   (front_l
    :reader front_l
    :initarg :front_l
    :type cl:float
    :initform 0.0))
)

(cl:defclass Distance (<Distance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Distance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Distance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sharps-msg:<Distance> is deprecated: use sharps-msg:Distance instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:header-val is deprecated.  Use sharps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rear-val :lambda-list '(m))
(cl:defmethod rear-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:rear-val is deprecated.  Use sharps-msg:rear instead.")
  (rear m))

(cl:ensure-generic-function 'front-val :lambda-list '(m))
(cl:defmethod front-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:front-val is deprecated.  Use sharps-msg:front instead.")
  (front m))

(cl:ensure-generic-function 'rear_r-val :lambda-list '(m))
(cl:defmethod rear_r-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:rear_r-val is deprecated.  Use sharps-msg:rear_r instead.")
  (rear_r m))

(cl:ensure-generic-function 'front_r-val :lambda-list '(m))
(cl:defmethod front_r-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:front_r-val is deprecated.  Use sharps-msg:front_r instead.")
  (front_r m))

(cl:ensure-generic-function 'rear_l-val :lambda-list '(m))
(cl:defmethod rear_l-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:rear_l-val is deprecated.  Use sharps-msg:rear_l instead.")
  (rear_l m))

(cl:ensure-generic-function 'front_l-val :lambda-list '(m))
(cl:defmethod front_l-val ((m <Distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sharps-msg:front_l-val is deprecated.  Use sharps-msg:front_l instead.")
  (front_l m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Distance>) ostream)
  "Serializes a message object of type '<Distance>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Distance>) istream)
  "Deserializes a message object of type '<Distance>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_l) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_l) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Distance>)))
  "Returns string type for a message object of type '<Distance>"
  "sharps/Distance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Distance)))
  "Returns string type for a message object of type 'Distance"
  "sharps/Distance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Distance>)))
  "Returns md5sum for a message object of type '<Distance>"
  "939ab581b431e3c1c016caf7e25338fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Distance)))
  "Returns md5sum for a message object of type 'Distance"
  "939ab581b431e3c1c016caf7e25338fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Distance>)))
  "Returns full string definition for message of type '<Distance>"
  (cl:format cl:nil "Header header~%float32 rear~%float32 front~%float32 rear_r~%float32 front_r~%float32 rear_l~%float32 front_l~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Distance)))
  "Returns full string definition for message of type 'Distance"
  (cl:format cl:nil "Header header~%float32 rear~%float32 front~%float32 rear_r~%float32 front_r~%float32 rear_l~%float32 front_l~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Distance>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Distance>))
  "Converts a ROS message object to a list"
  (cl:list 'Distance
    (cl:cons ':header (header msg))
    (cl:cons ':rear (rear msg))
    (cl:cons ':front (front msg))
    (cl:cons ':rear_r (rear_r msg))
    (cl:cons ':front_r (front_r msg))
    (cl:cons ':rear_l (rear_l msg))
    (cl:cons ':front_l (front_l msg))
))
