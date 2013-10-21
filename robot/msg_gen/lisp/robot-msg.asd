
(cl:in-package :asdf)

(defsystem "robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EKF" :depends-on ("_package_EKF"))
    (:file "_package_EKF" :depends-on ("_package"))
  ))