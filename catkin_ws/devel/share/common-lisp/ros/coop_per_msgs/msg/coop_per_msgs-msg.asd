
(cl:in-package :asdf)

(defsystem "coop_per_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectDetection" :depends-on ("_package_ObjectDetection"))
    (:file "_package_ObjectDetection" :depends-on ("_package"))
    (:file "ObjectDetectionFrame" :depends-on ("_package_ObjectDetectionFrame"))
    (:file "_package_ObjectDetectionFrame" :depends-on ("_package"))
  ))