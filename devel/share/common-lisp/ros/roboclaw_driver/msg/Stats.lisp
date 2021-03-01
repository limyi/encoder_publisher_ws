; Auto-generated. Do not edit!


(cl:in-package roboclaw_driver-msg)


;//! \htmlinclude Stats.msg.html

(cl:defclass <Stats> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (m1_enc_val
    :reader m1_enc_val
    :initarg :m1_enc_val
    :type cl:integer
    :initform 0)
   (m2_enc_val
    :reader m2_enc_val
    :initarg :m2_enc_val
    :type cl:integer
    :initform 0)
   (m1_enc_qpps
    :reader m1_enc_qpps
    :initarg :m1_enc_qpps
    :type cl:integer
    :initform 0)
   (m2_enc_qpps
    :reader m2_enc_qpps
    :initarg :m2_enc_qpps
    :type cl:integer
    :initform 0))
)

(cl:defclass Stats (<Stats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboclaw_driver-msg:<Stats> is deprecated: use roboclaw_driver-msg:Stats instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:header-val is deprecated.  Use roboclaw_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'm1_enc_val-val :lambda-list '(m))
(cl:defmethod m1_enc_val-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m1_enc_val-val is deprecated.  Use roboclaw_driver-msg:m1_enc_val instead.")
  (m1_enc_val m))

(cl:ensure-generic-function 'm2_enc_val-val :lambda-list '(m))
(cl:defmethod m2_enc_val-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m2_enc_val-val is deprecated.  Use roboclaw_driver-msg:m2_enc_val instead.")
  (m2_enc_val m))

(cl:ensure-generic-function 'm1_enc_qpps-val :lambda-list '(m))
(cl:defmethod m1_enc_qpps-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m1_enc_qpps-val is deprecated.  Use roboclaw_driver-msg:m1_enc_qpps instead.")
  (m1_enc_qpps m))

(cl:ensure-generic-function 'm2_enc_qpps-val :lambda-list '(m))
(cl:defmethod m2_enc_qpps-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m2_enc_qpps-val is deprecated.  Use roboclaw_driver-msg:m2_enc_qpps instead.")
  (m2_enc_qpps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stats>) ostream)
  "Serializes a message object of type '<Stats>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'm1_enc_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'm2_enc_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'm1_enc_qpps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'm2_enc_qpps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stats>) istream)
  "Deserializes a message object of type '<Stats>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm1_enc_val) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm2_enc_val) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm1_enc_qpps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm2_enc_qpps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stats>)))
  "Returns string type for a message object of type '<Stats>"
  "roboclaw_driver/Stats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stats)))
  "Returns string type for a message object of type 'Stats"
  "roboclaw_driver/Stats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stats>)))
  "Returns md5sum for a message object of type '<Stats>"
  "210b25d19a34293aed84261a946d88a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stats)))
  "Returns md5sum for a message object of type 'Stats"
  "210b25d19a34293aed84261a946d88a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stats>)))
  "Returns full string definition for message of type '<Stats>"
  (cl:format cl:nil "# std_msgs/Header ~%Header header~%~%# Motor 1 & 2 encoder values~%int32 m1_enc_val~%int32 m2_enc_val~%~%# Motor 1 & 2 speed values in QPPS (+/-)~%int32 m1_enc_qpps~%int32 m2_enc_qpps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stats)))
  "Returns full string definition for message of type 'Stats"
  (cl:format cl:nil "# std_msgs/Header ~%Header header~%~%# Motor 1 & 2 encoder values~%int32 m1_enc_val~%int32 m2_enc_val~%~%# Motor 1 & 2 speed values in QPPS (+/-)~%int32 m1_enc_qpps~%int32 m2_enc_qpps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stats>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stats>))
  "Converts a ROS message object to a list"
  (cl:list 'Stats
    (cl:cons ':header (header msg))
    (cl:cons ':m1_enc_val (m1_enc_val msg))
    (cl:cons ':m2_enc_val (m2_enc_val msg))
    (cl:cons ':m1_enc_qpps (m1_enc_qpps msg))
    (cl:cons ':m2_enc_qpps (m2_enc_qpps msg))
))
