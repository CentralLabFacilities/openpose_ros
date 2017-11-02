; Auto-generated. Do not edit!


(cl:in-package openpose_ros-msg)


;//! \htmlinclude PersonDetection.msg.html

(cl:defclass <PersonDetection> (roslisp-msg-protocol:ros-message)
  ((Nose
    :reader Nose
    :initarg :Nose
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (Neck
    :reader Neck
    :initarg :Neck
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RShoulder
    :reader RShoulder
    :initarg :RShoulder
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RElbow
    :reader RElbow
    :initarg :RElbow
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RWrist
    :reader RWrist
    :initarg :RWrist
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LShoulder
    :reader LShoulder
    :initarg :LShoulder
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LElbow
    :reader LElbow
    :initarg :LElbow
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LWrist
    :reader LWrist
    :initarg :LWrist
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RHip
    :reader RHip
    :initarg :RHip
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RKnee
    :reader RKnee
    :initarg :RKnee
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (RAnkle
    :reader RAnkle
    :initarg :RAnkle
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LHip
    :reader LHip
    :initarg :LHip
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LKnee
    :reader LKnee
    :initarg :LKnee
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LAnkle
    :reader LAnkle
    :initarg :LAnkle
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (REye
    :reader REye
    :initarg :REye
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LEye
    :reader LEye
    :initarg :LEye
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (REar
    :reader REar
    :initarg :REar
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (LEar
    :reader LEar
    :initarg :LEar
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection))
   (Chest
    :reader Chest
    :initarg :Chest
    :type openpose_ros-msg:BodyPartDetection
    :initform (cl:make-instance 'openpose_ros-msg:BodyPartDetection)))
)

(cl:defclass PersonDetection (<PersonDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros-msg:<PersonDetection> is deprecated: use openpose_ros-msg:PersonDetection instead.")))

(cl:ensure-generic-function 'Nose-val :lambda-list '(m))
(cl:defmethod Nose-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:Nose-val is deprecated.  Use openpose_ros-msg:Nose instead.")
  (Nose m))

(cl:ensure-generic-function 'Neck-val :lambda-list '(m))
(cl:defmethod Neck-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:Neck-val is deprecated.  Use openpose_ros-msg:Neck instead.")
  (Neck m))

(cl:ensure-generic-function 'RShoulder-val :lambda-list '(m))
(cl:defmethod RShoulder-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RShoulder-val is deprecated.  Use openpose_ros-msg:RShoulder instead.")
  (RShoulder m))

(cl:ensure-generic-function 'RElbow-val :lambda-list '(m))
(cl:defmethod RElbow-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RElbow-val is deprecated.  Use openpose_ros-msg:RElbow instead.")
  (RElbow m))

(cl:ensure-generic-function 'RWrist-val :lambda-list '(m))
(cl:defmethod RWrist-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RWrist-val is deprecated.  Use openpose_ros-msg:RWrist instead.")
  (RWrist m))

(cl:ensure-generic-function 'LShoulder-val :lambda-list '(m))
(cl:defmethod LShoulder-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LShoulder-val is deprecated.  Use openpose_ros-msg:LShoulder instead.")
  (LShoulder m))

(cl:ensure-generic-function 'LElbow-val :lambda-list '(m))
(cl:defmethod LElbow-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LElbow-val is deprecated.  Use openpose_ros-msg:LElbow instead.")
  (LElbow m))

(cl:ensure-generic-function 'LWrist-val :lambda-list '(m))
(cl:defmethod LWrist-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LWrist-val is deprecated.  Use openpose_ros-msg:LWrist instead.")
  (LWrist m))

(cl:ensure-generic-function 'RHip-val :lambda-list '(m))
(cl:defmethod RHip-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RHip-val is deprecated.  Use openpose_ros-msg:RHip instead.")
  (RHip m))

(cl:ensure-generic-function 'RKnee-val :lambda-list '(m))
(cl:defmethod RKnee-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RKnee-val is deprecated.  Use openpose_ros-msg:RKnee instead.")
  (RKnee m))

(cl:ensure-generic-function 'RAnkle-val :lambda-list '(m))
(cl:defmethod RAnkle-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:RAnkle-val is deprecated.  Use openpose_ros-msg:RAnkle instead.")
  (RAnkle m))

(cl:ensure-generic-function 'LHip-val :lambda-list '(m))
(cl:defmethod LHip-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LHip-val is deprecated.  Use openpose_ros-msg:LHip instead.")
  (LHip m))

(cl:ensure-generic-function 'LKnee-val :lambda-list '(m))
(cl:defmethod LKnee-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LKnee-val is deprecated.  Use openpose_ros-msg:LKnee instead.")
  (LKnee m))

(cl:ensure-generic-function 'LAnkle-val :lambda-list '(m))
(cl:defmethod LAnkle-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LAnkle-val is deprecated.  Use openpose_ros-msg:LAnkle instead.")
  (LAnkle m))

(cl:ensure-generic-function 'REye-val :lambda-list '(m))
(cl:defmethod REye-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:REye-val is deprecated.  Use openpose_ros-msg:REye instead.")
  (REye m))

(cl:ensure-generic-function 'LEye-val :lambda-list '(m))
(cl:defmethod LEye-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LEye-val is deprecated.  Use openpose_ros-msg:LEye instead.")
  (LEye m))

(cl:ensure-generic-function 'REar-val :lambda-list '(m))
(cl:defmethod REar-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:REar-val is deprecated.  Use openpose_ros-msg:REar instead.")
  (REar m))

(cl:ensure-generic-function 'LEar-val :lambda-list '(m))
(cl:defmethod LEar-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:LEar-val is deprecated.  Use openpose_ros-msg:LEar instead.")
  (LEar m))

(cl:ensure-generic-function 'Chest-val :lambda-list '(m))
(cl:defmethod Chest-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:Chest-val is deprecated.  Use openpose_ros-msg:Chest instead.")
  (Chest m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonDetection>) ostream)
  "Serializes a message object of type '<PersonDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Nose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Neck) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RShoulder) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RElbow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RWrist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LShoulder) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LElbow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LWrist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RHip) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RKnee) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'RAnkle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LHip) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LKnee) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LAnkle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'REye) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LEye) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'REar) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LEar) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Chest) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonDetection>) istream)
  "Deserializes a message object of type '<PersonDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Nose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Neck) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RShoulder) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RElbow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RWrist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LShoulder) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LElbow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LWrist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RHip) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RKnee) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'RAnkle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LHip) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LKnee) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LAnkle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'REye) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LEye) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'REar) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LEar) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Chest) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonDetection>)))
  "Returns string type for a message object of type '<PersonDetection>"
  "openpose_ros/PersonDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonDetection)))
  "Returns string type for a message object of type 'PersonDetection"
  "openpose_ros/PersonDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonDetection>)))
  "Returns md5sum for a message object of type '<PersonDetection>"
  "3a992f141e745a3f47d598e0bc953d79")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonDetection)))
  "Returns md5sum for a message object of type 'PersonDetection"
  "3a992f141e745a3f47d598e0bc953d79")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonDetection>)))
  "Returns full string definition for message of type '<PersonDetection>"
  (cl:format cl:nil "BodyPartDetection Nose~%BodyPartDetection Neck~%BodyPartDetection RShoulder~%BodyPartDetection RElbow~%BodyPartDetection RWrist~%BodyPartDetection LShoulder~%BodyPartDetection LElbow~%BodyPartDetection LWrist~%BodyPartDetection RHip~%BodyPartDetection RKnee~%BodyPartDetection RAnkle~%BodyPartDetection LHip~%BodyPartDetection LKnee~%BodyPartDetection LAnkle~%BodyPartDetection REye~%BodyPartDetection LEye~%BodyPartDetection REar~%BodyPartDetection LEar~%BodyPartDetection Chest~%~%================================================================================~%MSG: openpose_ros/BodyPartDetection~%geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonDetection)))
  "Returns full string definition for message of type 'PersonDetection"
  (cl:format cl:nil "BodyPartDetection Nose~%BodyPartDetection Neck~%BodyPartDetection RShoulder~%BodyPartDetection RElbow~%BodyPartDetection RWrist~%BodyPartDetection LShoulder~%BodyPartDetection LElbow~%BodyPartDetection LWrist~%BodyPartDetection RHip~%BodyPartDetection RKnee~%BodyPartDetection RAnkle~%BodyPartDetection LHip~%BodyPartDetection LKnee~%BodyPartDetection LAnkle~%BodyPartDetection REye~%BodyPartDetection LEye~%BodyPartDetection REar~%BodyPartDetection LEar~%BodyPartDetection Chest~%~%================================================================================~%MSG: openpose_ros/BodyPartDetection~%geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Nose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Neck))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RShoulder))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RElbow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RWrist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LShoulder))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LElbow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LWrist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RHip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RKnee))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'RAnkle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LHip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LKnee))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LAnkle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'REye))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LEye))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'REar))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LEar))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Chest))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonDetection
    (cl:cons ':Nose (Nose msg))
    (cl:cons ':Neck (Neck msg))
    (cl:cons ':RShoulder (RShoulder msg))
    (cl:cons ':RElbow (RElbow msg))
    (cl:cons ':RWrist (RWrist msg))
    (cl:cons ':LShoulder (LShoulder msg))
    (cl:cons ':LElbow (LElbow msg))
    (cl:cons ':LWrist (LWrist msg))
    (cl:cons ':RHip (RHip msg))
    (cl:cons ':RKnee (RKnee msg))
    (cl:cons ':RAnkle (RAnkle msg))
    (cl:cons ':LHip (LHip msg))
    (cl:cons ':LKnee (LKnee msg))
    (cl:cons ':LAnkle (LAnkle msg))
    (cl:cons ':REye (REye msg))
    (cl:cons ':LEye (LEye msg))
    (cl:cons ':REar (REar msg))
    (cl:cons ':LEar (LEar msg))
    (cl:cons ':Chest (Chest msg))
))
