
(cl:in-package :asdf)

(defsystem "roboclaw_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SpeedCommand" :depends-on ("_package_SpeedCommand"))
    (:file "_package_SpeedCommand" :depends-on ("_package"))
    (:file "Stats" :depends-on ("_package_Stats"))
    (:file "_package_Stats" :depends-on ("_package"))
  ))