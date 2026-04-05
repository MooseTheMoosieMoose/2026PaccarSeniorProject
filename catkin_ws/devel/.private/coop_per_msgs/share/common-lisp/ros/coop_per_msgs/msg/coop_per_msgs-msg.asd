
(cl:in-package :asdf)

(defsystem "coop_per_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ObjectDetection" :depends-on ("_package_ObjectDetection"))
    (:file "_package_ObjectDetection" :depends-on ("_package"))
  ))