;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros::DetectPeople)
  (if (not (find-package "OPENPOSE_ROS"))
    (make-package "OPENPOSE_ROS"))
  (shadow 'DetectPeople (find-package "OPENPOSE_ROS")))
(unless (find-package "OPENPOSE_ROS::DETECTPEOPLE")
  (make-package "OPENPOSE_ROS::DETECTPEOPLE"))
(unless (find-package "OPENPOSE_ROS::DETECTPEOPLEREQUEST")
  (make-package "OPENPOSE_ROS::DETECTPEOPLEREQUEST"))
(unless (find-package "OPENPOSE_ROS::DETECTPEOPLERESPONSE")
  (make-package "OPENPOSE_ROS::DETECTPEOPLERESPONSE"))

(in-package "ROS")





(defclass openpose_ros::DetectPeopleRequest
  :super ros::object
  :slots ())

(defmethod openpose_ros::DetectPeopleRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass openpose_ros::DetectPeopleResponse
  :super ros::object
  :slots (_people_list ))

(defmethod openpose_ros::DetectPeopleResponse
  (:init
   (&key
    ((:people_list __people_list) (let (r) (dotimes (i 0) (push (instance openpose_ros::PersonDetection :init) r)) r))
    )
   (send-super :init)
   (setq _people_list __people_list)
   self)
  (:people_list
   (&rest __people_list)
   (if (keywordp (car __people_list))
       (send* _people_list __people_list)
     (progn
       (if __people_list (setq _people_list (car __people_list)))
       _people_list)))
  (:serialization-length
   ()
   (+
    ;; openpose_ros/PersonDetection[] _people_list
    (apply #'+ (send-all _people_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openpose_ros/PersonDetection[] _people_list
     (write-long (length _people_list) s)
     (dolist (elem _people_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openpose_ros/PersonDetection[] _people_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _people_list (let (r) (dotimes (i n) (push (instance openpose_ros::PersonDetection :init) r)) r))
     (dolist (elem- _people_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass openpose_ros::DetectPeople
  :super ros::object
  :slots ())

(setf (get openpose_ros::DetectPeople :md5sum-) "f9061c1888cb3379ce3b5278d009dde5")
(setf (get openpose_ros::DetectPeople :datatype-) "openpose_ros/DetectPeople")
(setf (get openpose_ros::DetectPeople :request) openpose_ros::DetectPeopleRequest)
(setf (get openpose_ros::DetectPeople :response) openpose_ros::DetectPeopleResponse)

(defmethod openpose_ros::DetectPeopleRequest
  (:response () (instance openpose_ros::DetectPeopleResponse :init)))

(setf (get openpose_ros::DetectPeopleRequest :md5sum-) "f9061c1888cb3379ce3b5278d009dde5")
(setf (get openpose_ros::DetectPeopleRequest :datatype-) "openpose_ros/DetectPeopleRequest")
(setf (get openpose_ros::DetectPeopleRequest :definition-)
      "---
PersonDetection[] people_list


================================================================================
MSG: openpose_ros/PersonDetection
BodyPartDetection Nose
BodyPartDetection Neck
BodyPartDetection RShoulder
BodyPartDetection RElbow
BodyPartDetection RWrist
BodyPartDetection LShoulder
BodyPartDetection LElbow
BodyPartDetection LWrist
BodyPartDetection RHip
BodyPartDetection RKnee
BodyPartDetection RAnkle
BodyPartDetection LHip
BodyPartDetection LKnee
BodyPartDetection LAnkle
BodyPartDetection REye
BodyPartDetection LEye
BodyPartDetection REar
BodyPartDetection LEar
BodyPartDetection Chest

================================================================================
MSG: openpose_ros/BodyPartDetection
geometry_msgs/Point pos
float32 confidence

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get openpose_ros::DetectPeopleResponse :md5sum-) "f9061c1888cb3379ce3b5278d009dde5")
(setf (get openpose_ros::DetectPeopleResponse :datatype-) "openpose_ros/DetectPeopleResponse")
(setf (get openpose_ros::DetectPeopleResponse :definition-)
      "---
PersonDetection[] people_list


================================================================================
MSG: openpose_ros/PersonDetection
BodyPartDetection Nose
BodyPartDetection Neck
BodyPartDetection RShoulder
BodyPartDetection RElbow
BodyPartDetection RWrist
BodyPartDetection LShoulder
BodyPartDetection LElbow
BodyPartDetection LWrist
BodyPartDetection RHip
BodyPartDetection RKnee
BodyPartDetection RAnkle
BodyPartDetection LHip
BodyPartDetection LKnee
BodyPartDetection LAnkle
BodyPartDetection REye
BodyPartDetection LEye
BodyPartDetection REar
BodyPartDetection LEar
BodyPartDetection Chest

================================================================================
MSG: openpose_ros/BodyPartDetection
geometry_msgs/Point pos
float32 confidence

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :openpose_ros/DetectPeople "f9061c1888cb3379ce3b5278d009dde5")


