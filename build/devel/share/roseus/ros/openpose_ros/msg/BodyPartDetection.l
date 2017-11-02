;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros::BodyPartDetection)
  (if (not (find-package "OPENPOSE_ROS"))
    (make-package "OPENPOSE_ROS"))
  (shadow 'BodyPartDetection (find-package "OPENPOSE_ROS")))
(unless (find-package "OPENPOSE_ROS::BODYPARTDETECTION")
  (make-package "OPENPOSE_ROS::BODYPARTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude BodyPartDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass openpose_ros::BodyPartDetection
  :super ros::object
  :slots (_pos _confidence ))

(defmethod openpose_ros::BodyPartDetection
  (:init
   (&key
    ((:pos __pos) (instance geometry_msgs::Point :init))
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _confidence (float __confidence))
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _pos
    (send _pos :serialization-length)
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _pos
       (send _pos :serialize s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openpose_ros::BodyPartDetection :md5sum-) "d353c81ba0c136c3c45ae3297a81d8f3")
(setf (get openpose_ros::BodyPartDetection :datatype-) "openpose_ros/BodyPartDetection")
(setf (get openpose_ros::BodyPartDetection :definition-)
      "geometry_msgs/Point pos
float32 confidence

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :openpose_ros/BodyPartDetection "d353c81ba0c136c3c45ae3297a81d8f3")


