
(cl:in-package :asdf)

(defsystem "server_interface-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LockOn" :depends-on ("_package_LockOn"))
    (:file "_package_LockOn" :depends-on ("_package"))
    (:file "SubmitKamikaze" :depends-on ("_package_SubmitKamikaze"))
    (:file "_package_SubmitKamikaze" :depends-on ("_package"))
  ))