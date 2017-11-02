;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros::PersonDetection)
  (if (not (find-package "OPENPOSE_ROS"))
    (make-package "OPENPOSE_ROS"))
  (shadow 'PersonDetection (find-package "OPENPOSE_ROS")))
(unless (find-package "OPENPOSE_ROS::PERSONDETECTION")
  (make-package "OPENPOSE_ROS::PERSONDETECTION"))

(in-package "ROS")
;;//! \htmlinclude PersonDetection.msg.html


(defclass openpose_ros::PersonDetection
  :super ros::object
  :slots (_Nose _Neck _RShoulder _RElbow _RWrist _LShoulder _LElbow _LWrist _RHip _RKnee _RAnkle _LHip _LKnee _LAnkle _REye _LEye _REar _LEar _Chest ))

(defmethod openpose_ros::PersonDetection
  (:init
   (&key
    ((:Nose __Nose) (instance openpose_ros::BodyPartDetection :init))
    ((:Neck __Neck) (instance openpose_ros::BodyPartDetection :init))
    ((:RShoulder __RShoulder) (instance openpose_ros::BodyPartDetection :init))
    ((:RElbow __RElbow) (instance openpose_ros::BodyPartDetection :init))
    ((:RWrist __RWrist) (instance openpose_ros::BodyPartDetection :init))
    ((:LShoulder __LShoulder) (instance openpose_ros::BodyPartDetection :init))
    ((:LElbow __LElbow) (instance openpose_ros::BodyPartDetection :init))
    ((:LWrist __LWrist) (instance openpose_ros::BodyPartDetection :init))
    ((:RHip __RHip) (instance openpose_ros::BodyPartDetection :init))
    ((:RKnee __RKnee) (instance openpose_ros::BodyPartDetection :init))
    ((:RAnkle __RAnkle) (instance openpose_ros::BodyPartDetection :init))
    ((:LHip __LHip) (instance openpose_ros::BodyPartDetection :init))
    ((:LKnee __LKnee) (instance openpose_ros::BodyPartDetection :init))
    ((:LAnkle __LAnkle) (instance openpose_ros::BodyPartDetection :init))
    ((:REye __REye) (instance openpose_ros::BodyPartDetection :init))
    ((:LEye __LEye) (instance openpose_ros::BodyPartDetection :init))
    ((:REar __REar) (instance openpose_ros::BodyPartDetection :init))
    ((:LEar __LEar) (instance openpose_ros::BodyPartDetection :init))
    ((:Chest __Chest) (instance openpose_ros::BodyPartDetection :init))
    )
   (send-super :init)
   (setq _Nose __Nose)
   (setq _Neck __Neck)
   (setq _RShoulder __RShoulder)
   (setq _RElbow __RElbow)
   (setq _RWrist __RWrist)
   (setq _LShoulder __LShoulder)
   (setq _LElbow __LElbow)
   (setq _LWrist __LWrist)
   (setq _RHip __RHip)
   (setq _RKnee __RKnee)
   (setq _RAnkle __RAnkle)
   (setq _LHip __LHip)
   (setq _LKnee __LKnee)
   (setq _LAnkle __LAnkle)
   (setq _REye __REye)
   (setq _LEye __LEye)
   (setq _REar __REar)
   (setq _LEar __LEar)
   (setq _Chest __Chest)
   self)
  (:Nose
   (&rest __Nose)
   (if (keywordp (car __Nose))
       (send* _Nose __Nose)
     (progn
       (if __Nose (setq _Nose (car __Nose)))
       _Nose)))
  (:Neck
   (&rest __Neck)
   (if (keywordp (car __Neck))
       (send* _Neck __Neck)
     (progn
       (if __Neck (setq _Neck (car __Neck)))
       _Neck)))
  (:RShoulder
   (&rest __RShoulder)
   (if (keywordp (car __RShoulder))
       (send* _RShoulder __RShoulder)
     (progn
       (if __RShoulder (setq _RShoulder (car __RShoulder)))
       _RShoulder)))
  (:RElbow
   (&rest __RElbow)
   (if (keywordp (car __RElbow))
       (send* _RElbow __RElbow)
     (progn
       (if __RElbow (setq _RElbow (car __RElbow)))
       _RElbow)))
  (:RWrist
   (&rest __RWrist)
   (if (keywordp (car __RWrist))
       (send* _RWrist __RWrist)
     (progn
       (if __RWrist (setq _RWrist (car __RWrist)))
       _RWrist)))
  (:LShoulder
   (&rest __LShoulder)
   (if (keywordp (car __LShoulder))
       (send* _LShoulder __LShoulder)
     (progn
       (if __LShoulder (setq _LShoulder (car __LShoulder)))
       _LShoulder)))
  (:LElbow
   (&rest __LElbow)
   (if (keywordp (car __LElbow))
       (send* _LElbow __LElbow)
     (progn
       (if __LElbow (setq _LElbow (car __LElbow)))
       _LElbow)))
  (:LWrist
   (&rest __LWrist)
   (if (keywordp (car __LWrist))
       (send* _LWrist __LWrist)
     (progn
       (if __LWrist (setq _LWrist (car __LWrist)))
       _LWrist)))
  (:RHip
   (&rest __RHip)
   (if (keywordp (car __RHip))
       (send* _RHip __RHip)
     (progn
       (if __RHip (setq _RHip (car __RHip)))
       _RHip)))
  (:RKnee
   (&rest __RKnee)
   (if (keywordp (car __RKnee))
       (send* _RKnee __RKnee)
     (progn
       (if __RKnee (setq _RKnee (car __RKnee)))
       _RKnee)))
  (:RAnkle
   (&rest __RAnkle)
   (if (keywordp (car __RAnkle))
       (send* _RAnkle __RAnkle)
     (progn
       (if __RAnkle (setq _RAnkle (car __RAnkle)))
       _RAnkle)))
  (:LHip
   (&rest __LHip)
   (if (keywordp (car __LHip))
       (send* _LHip __LHip)
     (progn
       (if __LHip (setq _LHip (car __LHip)))
       _LHip)))
  (:LKnee
   (&rest __LKnee)
   (if (keywordp (car __LKnee))
       (send* _LKnee __LKnee)
     (progn
       (if __LKnee (setq _LKnee (car __LKnee)))
       _LKnee)))
  (:LAnkle
   (&rest __LAnkle)
   (if (keywordp (car __LAnkle))
       (send* _LAnkle __LAnkle)
     (progn
       (if __LAnkle (setq _LAnkle (car __LAnkle)))
       _LAnkle)))
  (:REye
   (&rest __REye)
   (if (keywordp (car __REye))
       (send* _REye __REye)
     (progn
       (if __REye (setq _REye (car __REye)))
       _REye)))
  (:LEye
   (&rest __LEye)
   (if (keywordp (car __LEye))
       (send* _LEye __LEye)
     (progn
       (if __LEye (setq _LEye (car __LEye)))
       _LEye)))
  (:REar
   (&rest __REar)
   (if (keywordp (car __REar))
       (send* _REar __REar)
     (progn
       (if __REar (setq _REar (car __REar)))
       _REar)))
  (:LEar
   (&rest __LEar)
   (if (keywordp (car __LEar))
       (send* _LEar __LEar)
     (progn
       (if __LEar (setq _LEar (car __LEar)))
       _LEar)))
  (:Chest
   (&rest __Chest)
   (if (keywordp (car __Chest))
       (send* _Chest __Chest)
     (progn
       (if __Chest (setq _Chest (car __Chest)))
       _Chest)))
  (:serialization-length
   ()
   (+
    ;; openpose_ros/BodyPartDetection _Nose
    (send _Nose :serialization-length)
    ;; openpose_ros/BodyPartDetection _Neck
    (send _Neck :serialization-length)
    ;; openpose_ros/BodyPartDetection _RShoulder
    (send _RShoulder :serialization-length)
    ;; openpose_ros/BodyPartDetection _RElbow
    (send _RElbow :serialization-length)
    ;; openpose_ros/BodyPartDetection _RWrist
    (send _RWrist :serialization-length)
    ;; openpose_ros/BodyPartDetection _LShoulder
    (send _LShoulder :serialization-length)
    ;; openpose_ros/BodyPartDetection _LElbow
    (send _LElbow :serialization-length)
    ;; openpose_ros/BodyPartDetection _LWrist
    (send _LWrist :serialization-length)
    ;; openpose_ros/BodyPartDetection _RHip
    (send _RHip :serialization-length)
    ;; openpose_ros/BodyPartDetection _RKnee
    (send _RKnee :serialization-length)
    ;; openpose_ros/BodyPartDetection _RAnkle
    (send _RAnkle :serialization-length)
    ;; openpose_ros/BodyPartDetection _LHip
    (send _LHip :serialization-length)
    ;; openpose_ros/BodyPartDetection _LKnee
    (send _LKnee :serialization-length)
    ;; openpose_ros/BodyPartDetection _LAnkle
    (send _LAnkle :serialization-length)
    ;; openpose_ros/BodyPartDetection _REye
    (send _REye :serialization-length)
    ;; openpose_ros/BodyPartDetection _LEye
    (send _LEye :serialization-length)
    ;; openpose_ros/BodyPartDetection _REar
    (send _REar :serialization-length)
    ;; openpose_ros/BodyPartDetection _LEar
    (send _LEar :serialization-length)
    ;; openpose_ros/BodyPartDetection _Chest
    (send _Chest :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openpose_ros/BodyPartDetection _Nose
       (send _Nose :serialize s)
     ;; openpose_ros/BodyPartDetection _Neck
       (send _Neck :serialize s)
     ;; openpose_ros/BodyPartDetection _RShoulder
       (send _RShoulder :serialize s)
     ;; openpose_ros/BodyPartDetection _RElbow
       (send _RElbow :serialize s)
     ;; openpose_ros/BodyPartDetection _RWrist
       (send _RWrist :serialize s)
     ;; openpose_ros/BodyPartDetection _LShoulder
       (send _LShoulder :serialize s)
     ;; openpose_ros/BodyPartDetection _LElbow
       (send _LElbow :serialize s)
     ;; openpose_ros/BodyPartDetection _LWrist
       (send _LWrist :serialize s)
     ;; openpose_ros/BodyPartDetection _RHip
       (send _RHip :serialize s)
     ;; openpose_ros/BodyPartDetection _RKnee
       (send _RKnee :serialize s)
     ;; openpose_ros/BodyPartDetection _RAnkle
       (send _RAnkle :serialize s)
     ;; openpose_ros/BodyPartDetection _LHip
       (send _LHip :serialize s)
     ;; openpose_ros/BodyPartDetection _LKnee
       (send _LKnee :serialize s)
     ;; openpose_ros/BodyPartDetection _LAnkle
       (send _LAnkle :serialize s)
     ;; openpose_ros/BodyPartDetection _REye
       (send _REye :serialize s)
     ;; openpose_ros/BodyPartDetection _LEye
       (send _LEye :serialize s)
     ;; openpose_ros/BodyPartDetection _REar
       (send _REar :serialize s)
     ;; openpose_ros/BodyPartDetection _LEar
       (send _LEar :serialize s)
     ;; openpose_ros/BodyPartDetection _Chest
       (send _Chest :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openpose_ros/BodyPartDetection _Nose
     (send _Nose :deserialize buf ptr-) (incf ptr- (send _Nose :serialization-length))
   ;; openpose_ros/BodyPartDetection _Neck
     (send _Neck :deserialize buf ptr-) (incf ptr- (send _Neck :serialization-length))
   ;; openpose_ros/BodyPartDetection _RShoulder
     (send _RShoulder :deserialize buf ptr-) (incf ptr- (send _RShoulder :serialization-length))
   ;; openpose_ros/BodyPartDetection _RElbow
     (send _RElbow :deserialize buf ptr-) (incf ptr- (send _RElbow :serialization-length))
   ;; openpose_ros/BodyPartDetection _RWrist
     (send _RWrist :deserialize buf ptr-) (incf ptr- (send _RWrist :serialization-length))
   ;; openpose_ros/BodyPartDetection _LShoulder
     (send _LShoulder :deserialize buf ptr-) (incf ptr- (send _LShoulder :serialization-length))
   ;; openpose_ros/BodyPartDetection _LElbow
     (send _LElbow :deserialize buf ptr-) (incf ptr- (send _LElbow :serialization-length))
   ;; openpose_ros/BodyPartDetection _LWrist
     (send _LWrist :deserialize buf ptr-) (incf ptr- (send _LWrist :serialization-length))
   ;; openpose_ros/BodyPartDetection _RHip
     (send _RHip :deserialize buf ptr-) (incf ptr- (send _RHip :serialization-length))
   ;; openpose_ros/BodyPartDetection _RKnee
     (send _RKnee :deserialize buf ptr-) (incf ptr- (send _RKnee :serialization-length))
   ;; openpose_ros/BodyPartDetection _RAnkle
     (send _RAnkle :deserialize buf ptr-) (incf ptr- (send _RAnkle :serialization-length))
   ;; openpose_ros/BodyPartDetection _LHip
     (send _LHip :deserialize buf ptr-) (incf ptr- (send _LHip :serialization-length))
   ;; openpose_ros/BodyPartDetection _LKnee
     (send _LKnee :deserialize buf ptr-) (incf ptr- (send _LKnee :serialization-length))
   ;; openpose_ros/BodyPartDetection _LAnkle
     (send _LAnkle :deserialize buf ptr-) (incf ptr- (send _LAnkle :serialization-length))
   ;; openpose_ros/BodyPartDetection _REye
     (send _REye :deserialize buf ptr-) (incf ptr- (send _REye :serialization-length))
   ;; openpose_ros/BodyPartDetection _LEye
     (send _LEye :deserialize buf ptr-) (incf ptr- (send _LEye :serialization-length))
   ;; openpose_ros/BodyPartDetection _REar
     (send _REar :deserialize buf ptr-) (incf ptr- (send _REar :serialization-length))
   ;; openpose_ros/BodyPartDetection _LEar
     (send _LEar :deserialize buf ptr-) (incf ptr- (send _LEar :serialization-length))
   ;; openpose_ros/BodyPartDetection _Chest
     (send _Chest :deserialize buf ptr-) (incf ptr- (send _Chest :serialization-length))
   ;;
   self)
  )

(setf (get openpose_ros::PersonDetection :md5sum-) "3a992f141e745a3f47d598e0bc953d79")
(setf (get openpose_ros::PersonDetection :datatype-) "openpose_ros/PersonDetection")
(setf (get openpose_ros::PersonDetection :definition-)
      "BodyPartDetection Nose
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



(provide :openpose_ros/PersonDetection "3a992f141e745a3f47d598e0bc953d79")


