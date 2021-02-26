; Auto-generated. Do not edit!


(cl:in-package panthera_locomotion-srv)


;//! \htmlinclude Status-request.msg.html

(cl:defclass <Status-request> (roslisp-msg-protocol:ros-message)
  ((reconfig
    :reader reconfig
    :initarg :reconfig
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Status-request (<Status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-srv:<Status-request> is deprecated: use panthera_locomotion-srv:Status-request instead.")))

(cl:ensure-generic-function 'reconfig-val :lambda-list '(m))
(cl:defmethod reconfig-val ((m <Status-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:reconfig-val is deprecated.  Use panthera_locomotion-srv:reconfig instead.")
  (reconfig m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status-request>) ostream)
  "Serializes a message object of type '<Status-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reconfig) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status-request>) istream)
  "Deserializes a message object of type '<Status-request>"
    (cl:setf (cl:slot-value msg 'reconfig) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status-request>)))
  "Returns string type for a service object of type '<Status-request>"
  "panthera_locomotion/StatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status-request)))
  "Returns string type for a service object of type 'Status-request"
  "panthera_locomotion/StatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status-request>)))
  "Returns md5sum for a message object of type '<Status-request>"
  "6eac67a75eb7db125401139ae8ae1357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status-request)))
  "Returns md5sum for a message object of type 'Status-request"
  "6eac67a75eb7db125401139ae8ae1357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status-request>)))
  "Returns full string definition for message of type '<Status-request>"
  (cl:format cl:nil "bool reconfig~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status-request)))
  "Returns full string definition for message of type 'Status-request"
  (cl:format cl:nil "bool reconfig~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Status-request
    (cl:cons ':reconfig (reconfig msg))
))
;//! \htmlinclude Status-response.msg.html

(cl:defclass <Status-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass Status-response (<Status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-srv:<Status-response> is deprecated: use panthera_locomotion-srv:Status-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:status-val is deprecated.  Use panthera_locomotion-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:speed-val is deprecated.  Use panthera_locomotion-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status-response>) ostream)
  "Serializes a message object of type '<Status-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status-response>) istream)
  "Deserializes a message object of type '<Status-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status-response>)))
  "Returns string type for a service object of type '<Status-response>"
  "panthera_locomotion/StatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status-response)))
  "Returns string type for a service object of type 'Status-response"
  "panthera_locomotion/StatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status-response>)))
  "Returns md5sum for a message object of type '<Status-response>"
  "6eac67a75eb7db125401139ae8ae1357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status-response)))
  "Returns md5sum for a message object of type 'Status-response"
  "6eac67a75eb7db125401139ae8ae1357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status-response>)))
  "Returns full string definition for message of type '<Status-response>"
  (cl:format cl:nil "bool status~%int64 speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status-response)))
  "Returns full string definition for message of type 'Status-response"
  (cl:format cl:nil "bool status~%int64 speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Status-response
    (cl:cons ':status (status msg))
    (cl:cons ':speed (speed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Status)))
  'Status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Status)))
  'Status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status)))
  "Returns string type for a service object of type '<Status>"
  "panthera_locomotion/Status")