
(cl:in-package :asdf)

(defsystem "robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Rotate" :depends-on ("_package_Rotate"))
    (:file "_package_Rotate" :depends-on ("_package"))
    (:file "EKF" :depends-on ("_package_EKF"))
    (:file "_package_EKF" :depends-on ("_package"))
  ))