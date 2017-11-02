
(cl:in-package :asdf)

(defsystem "openpose_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :openpose_ros-msg
)
  :components ((:file "_package")
    (:file "DetectPeople" :depends-on ("_package_DetectPeople"))
    (:file "_package_DetectPeople" :depends-on ("_package"))
  ))