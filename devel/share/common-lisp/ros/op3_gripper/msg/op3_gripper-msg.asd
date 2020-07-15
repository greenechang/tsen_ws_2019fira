
(cl:in-package :asdf)

(defsystem "op3_gripper-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GripperPosition" :depends-on ("_package_GripperPosition"))
    (:file "_package_GripperPosition" :depends-on ("_package"))
  ))