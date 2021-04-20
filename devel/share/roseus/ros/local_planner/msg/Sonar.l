;; Auto-generated. Do not edit!


(when (boundp 'local_planner::Sonar)
  (if (not (find-package "LOCAL_PLANNER"))
    (make-package "LOCAL_PLANNER"))
  (shadow 'Sonar (find-package "LOCAL_PLANNER")))
(unless (find-package "LOCAL_PLANNER::SONAR")
  (make-package "LOCAL_PLANNER::SONAR"))

(in-package "ROS")
;;//! \htmlinclude Sonar.msg.html


(defclass local_planner::Sonar
  :super ros::object
  :slots (_back_l _back_r _left_b _left_m _left_f _front_l _front_r _right_f _right_m _right_b ))

(defmethod local_planner::Sonar
  (:init
   (&key
    ((:back_l __back_l) 0.0)
    ((:back_r __back_r) 0.0)
    ((:left_b __left_b) 0.0)
    ((:left_m __left_m) 0.0)
    ((:left_f __left_f) 0.0)
    ((:front_l __front_l) 0.0)
    ((:front_r __front_r) 0.0)
    ((:right_f __right_f) 0.0)
    ((:right_m __right_m) 0.0)
    ((:right_b __right_b) 0.0)
    )
   (send-super :init)
   (setq _back_l (float __back_l))
   (setq _back_r (float __back_r))
   (setq _left_b (float __left_b))
   (setq _left_m (float __left_m))
   (setq _left_f (float __left_f))
   (setq _front_l (float __front_l))
   (setq _front_r (float __front_r))
   (setq _right_f (float __right_f))
   (setq _right_m (float __right_m))
   (setq _right_b (float __right_b))
   self)
  (:back_l
   (&optional __back_l)
   (if __back_l (setq _back_l __back_l)) _back_l)
  (:back_r
   (&optional __back_r)
   (if __back_r (setq _back_r __back_r)) _back_r)
  (:left_b
   (&optional __left_b)
   (if __left_b (setq _left_b __left_b)) _left_b)
  (:left_m
   (&optional __left_m)
   (if __left_m (setq _left_m __left_m)) _left_m)
  (:left_f
   (&optional __left_f)
   (if __left_f (setq _left_f __left_f)) _left_f)
  (:front_l
   (&optional __front_l)
   (if __front_l (setq _front_l __front_l)) _front_l)
  (:front_r
   (&optional __front_r)
   (if __front_r (setq _front_r __front_r)) _front_r)
  (:right_f
   (&optional __right_f)
   (if __right_f (setq _right_f __right_f)) _right_f)
  (:right_m
   (&optional __right_m)
   (if __right_m (setq _right_m __right_m)) _right_m)
  (:right_b
   (&optional __right_b)
   (if __right_b (setq _right_b __right_b)) _right_b)
  (:serialization-length
   ()
   (+
    ;; float64 _back_l
    8
    ;; float64 _back_r
    8
    ;; float64 _left_b
    8
    ;; float64 _left_m
    8
    ;; float64 _left_f
    8
    ;; float64 _front_l
    8
    ;; float64 _front_r
    8
    ;; float64 _right_f
    8
    ;; float64 _right_m
    8
    ;; float64 _right_b
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _back_l
       (sys::poke _back_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _back_r
       (sys::poke _back_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_b
       (sys::poke _left_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_m
       (sys::poke _left_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_f
       (sys::poke _left_f (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_l
       (sys::poke _front_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_r
       (sys::poke _front_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_f
       (sys::poke _right_f (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_m
       (sys::poke _right_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_b
       (sys::poke _right_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _back_l
     (setq _back_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _back_r
     (setq _back_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_b
     (setq _left_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_m
     (setq _left_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_f
     (setq _left_f (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_l
     (setq _front_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_r
     (setq _front_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_f
     (setq _right_f (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_m
     (setq _right_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_b
     (setq _right_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get local_planner::Sonar :md5sum-) "f867769555dffd428a6a95bc8cdaae9c")
(setf (get local_planner::Sonar :datatype-) "local_planner/Sonar")
(setf (get local_planner::Sonar :definition-)
      "float64 back_l
float64 back_r

float64 left_b
float64 left_m
float64 left_f

float64 front_l
float64 front_r

float64 right_f
float64 right_m
float64 right_b
")



(provide :local_planner/Sonar "f867769555dffd428a6a95bc8cdaae9c")


