
(cl:in-package :asdf)

(defsystem "openpose_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BodyPartDetection" :depends-on ("_package_BodyPartDetection"))
    (:file "_package_BodyPartDetection" :depends-on ("_package"))
    (:file "PersonDetection" :depends-on ("_package_PersonDetection"))
    (:file "_package_PersonDetection" :depends-on ("_package"))
  ))