;; Auto-generated. Do not edit!


(when (boundp 'panthera_locomotion::ICRsearch)
  (if (not (find-package "PANTHERA_LOCOMOTION"))
    (make-package "PANTHERA_LOCOMOTION"))
  (shadow 'ICRsearch (find-package "PANTHERA_LOCOMOTION")))
(unless (find-package "PANTHERA_LOCOMOTION::ICRSEARCH")
  (make-package "PANTHERA_LOCOMOTION::ICRSEARCH"))
(unless (find-package "PANTHERA_LOCOMOTION::ICRSEARCHREQUEST")
  (make-package "PANTHERA_LOCOMOTION::ICRSEARCHREQUEST"))
(unless (find-package "PANTHERA_LOCOMOTION::ICRSEARCHRESPONSE")
  (make-package "PANTHERA_LOCOMOTION::ICRSEARCHRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass panthera_locomotion::ICRsearchRequest
  :super ros::object
  :slots (_received_angle _turn_angle ))

(defmethod panthera_locomotion::ICRsearchRequest
  (:init
   (&key
    ((:received_angle __received_angle) nil)
    ((:turn_angle __turn_angle) 0.0)
    )
   (send-super :init)
   (setq _received_angle __received_angle)
   (setq _turn_angle (float __turn_angle))
   self)
  (:received_angle
   (&optional __received_angle)
   (if __received_angle (setq _received_angle __received_angle)) _received_angle)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:serialization-length
   ()
   (+
    ;; bool _received_angle
    1
    ;; float64 _turn_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _received_angle
       (if _received_angle (write-byte -1 s) (write-byte 0 s))
     ;; float64 _turn_angle
       (sys::poke _turn_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _received_angle
     (setq _received_angle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass panthera_locomotion::ICRsearchResponse
  :super ros::object
  :slots (_feasibility _wheel_angles _wheel_speeds ))

(defmethod panthera_locomotion::ICRsearchResponse
  (:init
   (&key
    ((:feasibility __feasibility) nil)
    ((:wheel_angles __wheel_angles) (instance geometry_msgs::Twist :init))
    ((:wheel_speeds __wheel_speeds) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _feasibility __feasibility)
   (setq _wheel_angles __wheel_angles)
   (setq _wheel_speeds __wheel_speeds)
   self)
  (:feasibility
   (&optional __feasibility)
   (if __feasibility (setq _feasibility __feasibility)) _feasibility)
  (:wheel_angles
   (&rest __wheel_angles)
   (if (keywordp (car __wheel_angles))
       (send* _wheel_angles __wheel_angles)
     (progn
       (if __wheel_angles (setq _wheel_angles (car __wheel_angles)))
       _wheel_angles)))
  (:wheel_speeds
   (&rest __wheel_speeds)
   (if (keywordp (car __wheel_speeds))
       (send* _wheel_speeds __wheel_speeds)
     (progn
       (if __wheel_speeds (setq _wheel_speeds (car __wheel_speeds)))
       _wheel_speeds)))
  (:serialization-length
   ()
   (+
    ;; bool _feasibility
    1
    ;; geometry_msgs/Twist _wheel_angles
    (send _wheel_angles :serialization-length)
    ;; geometry_msgs/Twist _wheel_speeds
    (send _wheel_speeds :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _feasibility
       (if _feasibility (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Twist _wheel_angles
       (send _wheel_angles :serialize s)
     ;; geometry_msgs/Twist _wheel_speeds
       (send _wheel_speeds :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _feasibility
     (setq _feasibility (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Twist _wheel_angles
     (send _wheel_angles :deserialize buf ptr-) (incf ptr- (send _wheel_angles :serialization-length))
   ;; geometry_msgs/Twist _wheel_speeds
     (send _wheel_speeds :deserialize buf ptr-) (incf ptr- (send _wheel_speeds :serialization-length))
   ;;
   self)
  )

(defclass panthera_locomotion::ICRsearch
  :super ros::object
  :slots ())

(setf (get panthera_locomotion::ICRsearch :md5sum-) "bcaf6636f1df5330595534eeebecd7f8")
(setf (get panthera_locomotion::ICRsearch :datatype-) "panthera_locomotion/ICRsearch")
(setf (get panthera_locomotion::ICRsearch :request) panthera_locomotion::ICRsearchRequest)
(setf (get panthera_locomotion::ICRsearch :response) panthera_locomotion::ICRsearchResponse)

(defmethod panthera_locomotion::ICRsearchRequest
  (:response () (instance panthera_locomotion::ICRsearchResponse :init)))

(setf (get panthera_locomotion::ICRsearchRequest :md5sum-) "bcaf6636f1df5330595534eeebecd7f8")
(setf (get panthera_locomotion::ICRsearchRequest :datatype-) "panthera_locomotion/ICRsearchRequest")
(setf (get panthera_locomotion::ICRsearchRequest :definition-)
      "bool received_angle
float64 turn_angle
---
bool feasibility
geometry_msgs/Twist wheel_angles
geometry_msgs/Twist wheel_speeds


================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")

(setf (get panthera_locomotion::ICRsearchResponse :md5sum-) "bcaf6636f1df5330595534eeebecd7f8")
(setf (get panthera_locomotion::ICRsearchResponse :datatype-) "panthera_locomotion/ICRsearchResponse")
(setf (get panthera_locomotion::ICRsearchResponse :definition-)
      "bool received_angle
float64 turn_angle
---
bool feasibility
geometry_msgs/Twist wheel_angles
geometry_msgs/Twist wheel_speeds


================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")



(provide :panthera_locomotion/ICRsearch "bcaf6636f1df5330595534eeebecd7f8")


