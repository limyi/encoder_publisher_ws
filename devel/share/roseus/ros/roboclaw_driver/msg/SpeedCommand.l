;; Auto-generated. Do not edit!


(when (boundp 'roboclaw_driver::SpeedCommand)
  (if (not (find-package "ROBOCLAW_DRIVER"))
    (make-package "ROBOCLAW_DRIVER"))
  (shadow 'SpeedCommand (find-package "ROBOCLAW_DRIVER")))
(unless (find-package "ROBOCLAW_DRIVER::SPEEDCOMMAND")
  (make-package "ROBOCLAW_DRIVER::SPEEDCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SpeedCommand.msg.html


(defclass roboclaw_driver::SpeedCommand
  :super ros::object
  :slots (_m1_qpps _m2_qpps _accel _max_secs ))

(defmethod roboclaw_driver::SpeedCommand
  (:init
   (&key
    ((:m1_qpps __m1_qpps) 0)
    ((:m2_qpps __m2_qpps) 0)
    ((:accel __accel) 0)
    ((:max_secs __max_secs) 0)
    )
   (send-super :init)
   (setq _m1_qpps (round __m1_qpps))
   (setq _m2_qpps (round __m2_qpps))
   (setq _accel (round __accel))
   (setq _max_secs (round __max_secs))
   self)
  (:m1_qpps
   (&optional __m1_qpps)
   (if __m1_qpps (setq _m1_qpps __m1_qpps)) _m1_qpps)
  (:m2_qpps
   (&optional __m2_qpps)
   (if __m2_qpps (setq _m2_qpps __m2_qpps)) _m2_qpps)
  (:accel
   (&optional __accel)
   (if __accel (setq _accel __accel)) _accel)
  (:max_secs
   (&optional __max_secs)
   (if __max_secs (setq _max_secs __max_secs)) _max_secs)
  (:serialization-length
   ()
   (+
    ;; int32 _m1_qpps
    4
    ;; int32 _m2_qpps
    4
    ;; uint32 _accel
    4
    ;; uint32 _max_secs
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _m1_qpps
       (write-long _m1_qpps s)
     ;; int32 _m2_qpps
       (write-long _m2_qpps s)
     ;; uint32 _accel
       (write-long _accel s)
     ;; uint32 _max_secs
       (write-long _max_secs s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _m1_qpps
     (setq _m1_qpps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _m2_qpps
     (setq _m2_qpps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _accel
     (setq _accel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _max_secs
     (setq _max_secs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get roboclaw_driver::SpeedCommand :md5sum-) "1217b4f680d01eb5008f99442806c05f")
(setf (get roboclaw_driver::SpeedCommand :datatype-) "roboclaw_driver/SpeedCommand")
(setf (get roboclaw_driver::SpeedCommand :definition-)
      "# Motor speed in QPPS
int32 m1_qpps
int32 m2_qpps

# Acceleration
uint32 accel

# Max seconds before automatically stopping
uint32 max_secs

")



(provide :roboclaw_driver/SpeedCommand "1217b4f680d01eb5008f99442806c05f")


