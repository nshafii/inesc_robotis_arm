
(cl:in-package :asdf)

(defsystem "robotis_controller_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControlWrite" :depends-on ("_package_ControlWrite"))
    (:file "_package_ControlWrite" :depends-on ("_package"))
    (:file "PublishPosition" :depends-on ("_package_PublishPosition"))
    (:file "_package_PublishPosition" :depends-on ("_package"))
    (:file "ControlTorque" :depends-on ("_package_ControlTorque"))
    (:file "_package_ControlTorque" :depends-on ("_package"))
  ))