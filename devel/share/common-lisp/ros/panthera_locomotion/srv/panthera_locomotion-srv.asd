
(cl:in-package :asdf)

(defsystem "panthera_locomotion-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ICRsearch" :depends-on ("_package_ICRsearch"))
    (:file "_package_ICRsearch" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
  ))