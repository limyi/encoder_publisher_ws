;; Auto-generated. Do not edit!


(when (boundp 'local_planner::CmapClear)
  (if (not (find-package "LOCAL_PLANNER"))
    (make-package "LOCAL_PLANNER"))
  (shadow 'CmapClear (find-package "LOCAL_PLANNER")))
(unless (find-package "LOCAL_PLANNER::CMAPCLEAR")
  (make-package "LOCAL_PLANNER::CMAPCLEAR"))

(in-package "ROS")
;;//! \htmlinclude CmapClear.msg.html


(defclass local_planner::CmapClear
  :super ros::object
  :slots (_right _up _left _back _radius ))

(defmethod local_planner::CmapClear
  (:init
   (&key
    ((:right __right) nil)
    ((:up __up) nil)
    ((:left __left) nil)
    ((:back __back) nil)
    ((:radius __radius) nil)
    )
   (send-super :init)
   (setq _right __right)
   (setq _up __up)
   (setq _left __left)
   (setq _back __back)
   (setq _radius __radius)
   self)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:up
   (&optional __up)
   (if __up (setq _up __up)) _up)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:back
   (&optional __back)
   (if __back (setq _back __back)) _back)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; bool _right
    1
    ;; bool _up
    1
    ;; bool _left
    1
    ;; bool _back
    1
    ;; bool _radius
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;; bool _up
       (if _up (write-byte -1 s) (write-byte 0 s))
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;; bool _back
       (if _back (write-byte -1 s) (write-byte 0 s))
     ;; bool _radius
       (if _radius (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _up
     (setq _up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _back
     (setq _back (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _radius
     (setq _radius (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get local_planner::CmapClear :md5sum-) "fcdcf2c8de4d9bb6062a42facde1b732")
(setf (get local_planner::CmapClear :datatype-) "local_planner/CmapClear")
(setf (get local_planner::CmapClear :definition-)
      "bool right
bool up
bool left
bool back
bool radius

")



(provide :local_planner/CmapClear "fcdcf2c8de4d9bb6062a42facde1b732")


