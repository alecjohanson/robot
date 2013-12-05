
(cl:in-package :asdf)

(defsystem "sharps-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Distance" :depends-on ("_package_Distance"))
    (:file "_package_Distance" :depends-on ("_package"))
  ))