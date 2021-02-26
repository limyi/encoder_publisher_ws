;; Auto-generated. Do not edit!


(when (boundp 'panthera_locomotion::Status)
  (if (not (find-package "PANTHERA_LOCOMOTION"))
    (make-package "PANTHERA_LOCOMOTION"))
  (shadow 'Status (find-package "PANTHERA_LOCOMOTION")))
(unless (find-package "PANTHERA_LOCOMOTION::STATUS")
  (make-package "PANTHERA_LOCOMOTION::STATUS"))
(unless (find-package "PANTHERA_LOCOMOTION::STATUSREQUEST")
  (make-package "PANTHERA_LOCOMOTION::STATUSREQUEST"))
(unless (find-package "PANTHERA_LOCOMOTION::STATUSRESPONSE")
  (make-package "PANTHERA_LOCOMOTION::STATUSRESPONSE"))

(in-package "ROS")





(defclass panthera_locomotion::StatusRequest
  :super ros::object
  :slots (_reconfig ))

(defmethod panthera_locomotion::StatusRequest
  (:init
   (&key
    ((:reconfig __reconfig) nil)
    )
   (send-super :init)
   (setq _reconfig __reconfig)
   self)
  (:reconfig
   (&optional __reconfig)
   (if __reconfig (setq _reconfig __reconfig)) _reconfig)
  (:serialization-length
   ()
   (+
    ;; bool _reconfig
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _reconfig
       (if _reconfig (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _reconfig
     (setq _reconfig (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass panthera_locomotion::StatusResponse
  :super ros::object
  :slots (_status _speed ))

(defmethod panthera_locomotion::StatusResponse
  (:init
   (&key
    ((:status __status) nil)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _status __status)
   (setq _speed (round __speed))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ;; int64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;; int64 _speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _speed) (= (length (_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_speed . bv) 0) 0) s)
              (write-long (ash (elt (_speed . bv) 1) -1) s))
             ((and (class _speed) (= (length (_speed . bv)) 1)) ;; big1
              (write-long (elt (_speed . bv) 0) s)
              (write-long (if (>= _speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _speed s)(write-long (if (>= _speed 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _speed
#+(or :alpha :irix6 :x86_64)
      (setf _speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass panthera_locomotion::Status
  :super ros::object
  :slots ())

(setf (get panthera_locomotion::Status :md5sum-) "6eac67a75eb7db125401139ae8ae1357")
(setf (get panthera_locomotion::Status :datatype-) "panthera_locomotion/Status")
(setf (get panthera_locomotion::Status :request) panthera_locomotion::StatusRequest)
(setf (get panthera_locomotion::Status :response) panthera_locomotion::StatusResponse)

(defmethod panthera_locomotion::StatusRequest
  (:response () (instance panthera_locomotion::StatusResponse :init)))

(setf (get panthera_locomotion::StatusRequest :md5sum-) "6eac67a75eb7db125401139ae8ae1357")
(setf (get panthera_locomotion::StatusRequest :datatype-) "panthera_locomotion/StatusRequest")
(setf (get panthera_locomotion::StatusRequest :definition-)
      "bool reconfig
---
bool status
int64 speed

")

(setf (get panthera_locomotion::StatusResponse :md5sum-) "6eac67a75eb7db125401139ae8ae1357")
(setf (get panthera_locomotion::StatusResponse :datatype-) "panthera_locomotion/StatusResponse")
(setf (get panthera_locomotion::StatusResponse :definition-)
      "bool reconfig
---
bool status
int64 speed

")



(provide :panthera_locomotion/Status "6eac67a75eb7db125401139ae8ae1357")


