;; Auto-generated. Do not edit!


(when (boundp 'roboclaw_driver::Stats)
  (if (not (find-package "ROBOCLAW_DRIVER"))
    (make-package "ROBOCLAW_DRIVER"))
  (shadow 'Stats (find-package "ROBOCLAW_DRIVER")))
(unless (find-package "ROBOCLAW_DRIVER::STATS")
  (make-package "ROBOCLAW_DRIVER::STATS"))

(in-package "ROS")
;;//! \htmlinclude Stats.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass roboclaw_driver::Stats
  :super ros::object
  :slots (_header _m1_enc_val _m2_enc_val _m1_enc_qpps _m2_enc_qpps ))

(defmethod roboclaw_driver::Stats
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:m1_enc_val __m1_enc_val) 0)
    ((:m2_enc_val __m2_enc_val) 0)
    ((:m1_enc_qpps __m1_enc_qpps) 0)
    ((:m2_enc_qpps __m2_enc_qpps) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _m1_enc_val (round __m1_enc_val))
   (setq _m2_enc_val (round __m2_enc_val))
   (setq _m1_enc_qpps (round __m1_enc_qpps))
   (setq _m2_enc_qpps (round __m2_enc_qpps))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:m1_enc_val
   (&optional __m1_enc_val)
   (if __m1_enc_val (setq _m1_enc_val __m1_enc_val)) _m1_enc_val)
  (:m2_enc_val
   (&optional __m2_enc_val)
   (if __m2_enc_val (setq _m2_enc_val __m2_enc_val)) _m2_enc_val)
  (:m1_enc_qpps
   (&optional __m1_enc_qpps)
   (if __m1_enc_qpps (setq _m1_enc_qpps __m1_enc_qpps)) _m1_enc_qpps)
  (:m2_enc_qpps
   (&optional __m2_enc_qpps)
   (if __m2_enc_qpps (setq _m2_enc_qpps __m2_enc_qpps)) _m2_enc_qpps)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _m1_enc_val
    4
    ;; int32 _m2_enc_val
    4
    ;; int32 _m1_enc_qpps
    4
    ;; int32 _m2_enc_qpps
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _m1_enc_val
       (write-long _m1_enc_val s)
     ;; int32 _m2_enc_val
       (write-long _m2_enc_val s)
     ;; int32 _m1_enc_qpps
       (write-long _m1_enc_qpps s)
     ;; int32 _m2_enc_qpps
       (write-long _m2_enc_qpps s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _m1_enc_val
     (setq _m1_enc_val (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _m2_enc_val
     (setq _m2_enc_val (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _m1_enc_qpps
     (setq _m1_enc_qpps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _m2_enc_qpps
     (setq _m2_enc_qpps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get roboclaw_driver::Stats :md5sum-) "210b25d19a34293aed84261a946d88a5")
(setf (get roboclaw_driver::Stats :datatype-) "roboclaw_driver/Stats")
(setf (get roboclaw_driver::Stats :definition-)
      "# std_msgs/Header 
Header header

# Motor 1 & 2 encoder values
int32 m1_enc_val
int32 m2_enc_val

# Motor 1 & 2 speed values in QPPS (+/-)
int32 m1_enc_qpps
int32 m2_enc_qpps

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :roboclaw_driver/Stats "210b25d19a34293aed84261a946d88a5")


