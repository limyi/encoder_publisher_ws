; Auto-generated. Do not edit!


(cl:in-package local_planner-msg)


;//! \htmlinclude CmapClear.msg.html

(cl:defclass <CmapClear> (roslisp-msg-protocol:ros-message)
  ((right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil)
   (up
    :reader up
    :initarg :up
    :type cl:boolean
    :initform cl:nil)
   (left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (radius
    :reader radius
    :initarg :radius
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CmapClear (<CmapClear>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmapClear>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmapClear)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name local_planner-msg:<CmapClear> is deprecated: use local_planner-msg:CmapClear instead.")))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <CmapClear>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_planner-msg:right-val is deprecated.  Use local_planner-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'up-val :lambda-list '(m))
(cl:defmethod up-val ((m <CmapClear>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_planner-msg:up-val is deprecated.  Use local_planner-msg:up instead.")
  (up m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <CmapClear>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_planner-msg:left-val is deprecated.  Use local_planner-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <CmapClear>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_planner-msg:radius-val is deprecated.  Use local_planner-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmapClear>) ostream)
  "Serializes a message object of type '<CmapClear>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'up) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'radius) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmapClear>) istream)
  "Deserializes a message object of type '<CmapClear>"
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'radius) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmapClear>)))
  "Returns string type for a message object of type '<CmapClear>"
  "local_planner/CmapClear")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmapClear)))
  "Returns string type for a message object of type 'CmapClear"
  "local_planner/CmapClear")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmapClear>)))
  "Returns md5sum for a message object of type '<CmapClear>"
  "cbf4adf768e3f3ee806b11da1ab4d21f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmapClear)))
  "Returns md5sum for a message object of type 'CmapClear"
  "cbf4adf768e3f3ee806b11da1ab4d21f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmapClear>)))
  "Returns full string definition for message of type '<CmapClear>"
  (cl:format cl:nil "bool right~%bool up~%bool left~%bool radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmapClear)))
  "Returns full string definition for message of type 'CmapClear"
  (cl:format cl:nil "bool right~%bool up~%bool left~%bool radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmapClear>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmapClear>))
  "Converts a ROS message object to a list"
  (cl:list 'CmapClear
    (cl:cons ':right (right msg))
    (cl:cons ':up (up msg))
    (cl:cons ':left (left msg))
    (cl:cons ':radius (radius msg))
))
