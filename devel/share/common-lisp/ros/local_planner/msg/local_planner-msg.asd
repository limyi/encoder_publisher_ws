
(cl:in-package :asdf)

(defsystem "local_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CmapClear" :depends-on ("_package_CmapClear"))
    (:file "_package_CmapClear" :depends-on ("_package"))
  ))