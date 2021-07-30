; Auto-generated. Do not edit!


(cl:in-package panthera_locomotion-srv)


;//! \htmlinclude ICRsearch-request.msg.html

(cl:defclass <ICRsearch-request> (roslisp-msg-protocol:ros-message)
  ((received_angle
    :reader received_angle
    :initarg :received_angle
    :type cl:boolean
    :initform cl:nil)
   (turn_angle
    :reader turn_angle
    :initarg :turn_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass ICRsearch-request (<ICRsearch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ICRsearch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ICRsearch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-srv:<ICRsearch-request> is deprecated: use panthera_locomotion-srv:ICRsearch-request instead.")))

(cl:ensure-generic-function 'received_angle-val :lambda-list '(m))
(cl:defmethod received_angle-val ((m <ICRsearch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:received_angle-val is deprecated.  Use panthera_locomotion-srv:received_angle instead.")
  (received_angle m))

(cl:ensure-generic-function 'turn_angle-val :lambda-list '(m))
(cl:defmethod turn_angle-val ((m <ICRsearch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:turn_angle-val is deprecated.  Use panthera_locomotion-srv:turn_angle instead.")
  (turn_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ICRsearch-request>) ostream)
  "Serializes a message object of type '<ICRsearch-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'received_angle) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'turn_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ICRsearch-request>) istream)
  "Deserializes a message object of type '<ICRsearch-request>"
    (cl:setf (cl:slot-value msg 'received_angle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'turn_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ICRsearch-request>)))
  "Returns string type for a service object of type '<ICRsearch-request>"
  "panthera_locomotion/ICRsearchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICRsearch-request)))
  "Returns string type for a service object of type 'ICRsearch-request"
  "panthera_locomotion/ICRsearchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ICRsearch-request>)))
  "Returns md5sum for a message object of type '<ICRsearch-request>"
  "bcaf6636f1df5330595534eeebecd7f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ICRsearch-request)))
  "Returns md5sum for a message object of type 'ICRsearch-request"
  "bcaf6636f1df5330595534eeebecd7f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ICRsearch-request>)))
  "Returns full string definition for message of type '<ICRsearch-request>"
  (cl:format cl:nil "bool received_angle~%float64 turn_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ICRsearch-request)))
  "Returns full string definition for message of type 'ICRsearch-request"
  (cl:format cl:nil "bool received_angle~%float64 turn_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ICRsearch-request>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ICRsearch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ICRsearch-request
    (cl:cons ':received_angle (received_angle msg))
    (cl:cons ':turn_angle (turn_angle msg))
))
;//! \htmlinclude ICRsearch-response.msg.html

(cl:defclass <ICRsearch-response> (roslisp-msg-protocol:ros-message)
  ((feasibility
    :reader feasibility
    :initarg :feasibility
    :type cl:boolean
    :initform cl:nil)
   (wheel_angles
    :reader wheel_angles
    :initarg :wheel_angles
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (wheel_speeds
    :reader wheel_speeds
    :initarg :wheel_speeds
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass ICRsearch-response (<ICRsearch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ICRsearch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ICRsearch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-srv:<ICRsearch-response> is deprecated: use panthera_locomotion-srv:ICRsearch-response instead.")))

(cl:ensure-generic-function 'feasibility-val :lambda-list '(m))
(cl:defmethod feasibility-val ((m <ICRsearch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:feasibility-val is deprecated.  Use panthera_locomotion-srv:feasibility instead.")
  (feasibility m))

(cl:ensure-generic-function 'wheel_angles-val :lambda-list '(m))
(cl:defmethod wheel_angles-val ((m <ICRsearch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:wheel_angles-val is deprecated.  Use panthera_locomotion-srv:wheel_angles instead.")
  (wheel_angles m))

(cl:ensure-generic-function 'wheel_speeds-val :lambda-list '(m))
(cl:defmethod wheel_speeds-val ((m <ICRsearch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-srv:wheel_speeds-val is deprecated.  Use panthera_locomotion-srv:wheel_speeds instead.")
  (wheel_speeds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ICRsearch-response>) ostream)
  "Serializes a message object of type '<ICRsearch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'feasibility) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wheel_angles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wheel_speeds) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ICRsearch-response>) istream)
  "Deserializes a message object of type '<ICRsearch-response>"
    (cl:setf (cl:slot-value msg 'feasibility) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wheel_angles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wheel_speeds) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ICRsearch-response>)))
  "Returns string type for a service object of type '<ICRsearch-response>"
  "panthera_locomotion/ICRsearchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICRsearch-response)))
  "Returns string type for a service object of type 'ICRsearch-response"
  "panthera_locomotion/ICRsearchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ICRsearch-response>)))
  "Returns md5sum for a message object of type '<ICRsearch-response>"
  "bcaf6636f1df5330595534eeebecd7f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ICRsearch-response)))
  "Returns md5sum for a message object of type 'ICRsearch-response"
  "bcaf6636f1df5330595534eeebecd7f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ICRsearch-response>)))
  "Returns full string definition for message of type '<ICRsearch-response>"
  (cl:format cl:nil "bool feasibility~%geometry_msgs/Twist wheel_angles~%geometry_msgs/Twist wheel_speeds~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ICRsearch-response)))
  "Returns full string definition for message of type 'ICRsearch-response"
  (cl:format cl:nil "bool feasibility~%geometry_msgs/Twist wheel_angles~%geometry_msgs/Twist wheel_speeds~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ICRsearch-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wheel_angles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wheel_speeds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ICRsearch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ICRsearch-response
    (cl:cons ':feasibility (feasibility msg))
    (cl:cons ':wheel_angles (wheel_angles msg))
    (cl:cons ':wheel_speeds (wheel_speeds msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ICRsearch)))
  'ICRsearch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ICRsearch)))
  'ICRsearch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICRsearch)))
  "Returns string type for a service object of type '<ICRsearch>"
  "panthera_locomotion/ICRsearch")