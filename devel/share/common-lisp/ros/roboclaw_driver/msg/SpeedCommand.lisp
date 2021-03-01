; Auto-generated. Do not edit!


(cl:in-package roboclaw_driver-msg)


;//! \htmlinclude SpeedCommand.msg.html

(cl:defclass <SpeedCommand> (roslisp-msg-protocol:ros-message)
  ((m1_qpps
    :reader m1_qpps
    :initarg :m1_qpps
    :type cl:integer
    :initform 0)
   (m2_qpps
    :reader m2_qpps
    :initarg :m2_qpps
    :type cl:integer
    :initform 0)
   (accel
    :reader accel
    :initarg :accel
    :type cl:integer
    :initform 0)
   (max_secs
    :reader max_secs
    :initarg :max_secs
    :type cl:integer
    :initform 0))
)

(cl:defclass SpeedCommand (<SpeedCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeedCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeedCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roboclaw_driver-msg:<SpeedCommand> is deprecated: use roboclaw_driver-msg:SpeedCommand instead.")))

(cl:ensure-generic-function 'm1_qpps-val :lambda-list '(m))
(cl:defmethod m1_qpps-val ((m <SpeedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m1_qpps-val is deprecated.  Use roboclaw_driver-msg:m1_qpps instead.")
  (m1_qpps m))

(cl:ensure-generic-function 'm2_qpps-val :lambda-list '(m))
(cl:defmethod m2_qpps-val ((m <SpeedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:m2_qpps-val is deprecated.  Use roboclaw_driver-msg:m2_qpps instead.")
  (m2_qpps m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <SpeedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:accel-val is deprecated.  Use roboclaw_driver-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'max_secs-val :lambda-list '(m))
(cl:defmethod max_secs-val ((m <SpeedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roboclaw_driver-msg:max_secs-val is deprecated.  Use roboclaw_driver-msg:max_secs instead.")
  (max_secs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeedCommand>) ostream)
  "Serializes a message object of type '<SpeedCommand>"
  (cl:let* ((signed (cl:slot-value msg 'm1_qpps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'm2_qpps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'accel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'accel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'accel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'accel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_secs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_secs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_secs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_secs)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeedCommand>) istream)
  "Deserializes a message object of type '<SpeedCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm1_qpps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'm2_qpps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'accel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'accel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'accel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'accel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_secs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_secs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_secs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_secs)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeedCommand>)))
  "Returns string type for a message object of type '<SpeedCommand>"
  "roboclaw_driver/SpeedCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeedCommand)))
  "Returns string type for a message object of type 'SpeedCommand"
  "roboclaw_driver/SpeedCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeedCommand>)))
  "Returns md5sum for a message object of type '<SpeedCommand>"
  "1217b4f680d01eb5008f99442806c05f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeedCommand)))
  "Returns md5sum for a message object of type 'SpeedCommand"
  "1217b4f680d01eb5008f99442806c05f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeedCommand>)))
  "Returns full string definition for message of type '<SpeedCommand>"
  (cl:format cl:nil "# Motor speed in QPPS~%int32 m1_qpps~%int32 m2_qpps~%~%# Acceleration~%uint32 accel~%~%# Max seconds before automatically stopping~%uint32 max_secs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeedCommand)))
  "Returns full string definition for message of type 'SpeedCommand"
  (cl:format cl:nil "# Motor speed in QPPS~%int32 m1_qpps~%int32 m2_qpps~%~%# Acceleration~%uint32 accel~%~%# Max seconds before automatically stopping~%uint32 max_secs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeedCommand>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeedCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeedCommand
    (cl:cons ':m1_qpps (m1_qpps msg))
    (cl:cons ':m2_qpps (m2_qpps msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':max_secs (max_secs msg))
))
