; Auto-generated. Do not edit!


(cl:in-package openpose_ros-srv)


;//! \htmlinclude DetectPeople-request.msg.html

(cl:defclass <DetectPeople-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DetectPeople-request (<DetectPeople-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectPeople-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectPeople-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros-srv:<DetectPeople-request> is deprecated: use openpose_ros-srv:DetectPeople-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectPeople-request>) ostream)
  "Serializes a message object of type '<DetectPeople-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectPeople-request>) istream)
  "Deserializes a message object of type '<DetectPeople-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectPeople-request>)))
  "Returns string type for a service object of type '<DetectPeople-request>"
  "openpose_ros/DetectPeopleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectPeople-request)))
  "Returns string type for a service object of type 'DetectPeople-request"
  "openpose_ros/DetectPeopleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectPeople-request>)))
  "Returns md5sum for a message object of type '<DetectPeople-request>"
  "f9061c1888cb3379ce3b5278d009dde5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectPeople-request)))
  "Returns md5sum for a message object of type 'DetectPeople-request"
  "f9061c1888cb3379ce3b5278d009dde5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectPeople-request>)))
  "Returns full string definition for message of type '<DetectPeople-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectPeople-request)))
  "Returns full string definition for message of type 'DetectPeople-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectPeople-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectPeople-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectPeople-request
))
;//! \htmlinclude DetectPeople-response.msg.html

(cl:defclass <DetectPeople-response> (roslisp-msg-protocol:ros-message)
  ((people_list
    :reader people_list
    :initarg :people_list
    :type (cl:vector openpose_ros-msg:PersonDetection)
   :initform (cl:make-array 0 :element-type 'openpose_ros-msg:PersonDetection :initial-element (cl:make-instance 'openpose_ros-msg:PersonDetection))))
)

(cl:defclass DetectPeople-response (<DetectPeople-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectPeople-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectPeople-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros-srv:<DetectPeople-response> is deprecated: use openpose_ros-srv:DetectPeople-response instead.")))

(cl:ensure-generic-function 'people_list-val :lambda-list '(m))
(cl:defmethod people_list-val ((m <DetectPeople-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-srv:people_list-val is deprecated.  Use openpose_ros-srv:people_list instead.")
  (people_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectPeople-response>) ostream)
  "Serializes a message object of type '<DetectPeople-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'people_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'people_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectPeople-response>) istream)
  "Deserializes a message object of type '<DetectPeople-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'people_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'people_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros-msg:PersonDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectPeople-response>)))
  "Returns string type for a service object of type '<DetectPeople-response>"
  "openpose_ros/DetectPeopleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectPeople-response)))
  "Returns string type for a service object of type 'DetectPeople-response"
  "openpose_ros/DetectPeopleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectPeople-response>)))
  "Returns md5sum for a message object of type '<DetectPeople-response>"
  "f9061c1888cb3379ce3b5278d009dde5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectPeople-response)))
  "Returns md5sum for a message object of type 'DetectPeople-response"
  "f9061c1888cb3379ce3b5278d009dde5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectPeople-response>)))
  "Returns full string definition for message of type '<DetectPeople-response>"
  (cl:format cl:nil "PersonDetection[] people_list~%~%~%================================================================================~%MSG: openpose_ros/PersonDetection~%BodyPartDetection Nose~%BodyPartDetection Neck~%BodyPartDetection RShoulder~%BodyPartDetection RElbow~%BodyPartDetection RWrist~%BodyPartDetection LShoulder~%BodyPartDetection LElbow~%BodyPartDetection LWrist~%BodyPartDetection RHip~%BodyPartDetection RKnee~%BodyPartDetection RAnkle~%BodyPartDetection LHip~%BodyPartDetection LKnee~%BodyPartDetection LAnkle~%BodyPartDetection REye~%BodyPartDetection LEye~%BodyPartDetection REar~%BodyPartDetection LEar~%BodyPartDetection Chest~%~%================================================================================~%MSG: openpose_ros/BodyPartDetection~%geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectPeople-response)))
  "Returns full string definition for message of type 'DetectPeople-response"
  (cl:format cl:nil "PersonDetection[] people_list~%~%~%================================================================================~%MSG: openpose_ros/PersonDetection~%BodyPartDetection Nose~%BodyPartDetection Neck~%BodyPartDetection RShoulder~%BodyPartDetection RElbow~%BodyPartDetection RWrist~%BodyPartDetection LShoulder~%BodyPartDetection LElbow~%BodyPartDetection LWrist~%BodyPartDetection RHip~%BodyPartDetection RKnee~%BodyPartDetection RAnkle~%BodyPartDetection LHip~%BodyPartDetection LKnee~%BodyPartDetection LAnkle~%BodyPartDetection REye~%BodyPartDetection LEye~%BodyPartDetection REar~%BodyPartDetection LEar~%BodyPartDetection Chest~%~%================================================================================~%MSG: openpose_ros/BodyPartDetection~%geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectPeople-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'people_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectPeople-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectPeople-response
    (cl:cons ':people_list (people_list msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectPeople)))
  'DetectPeople-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectPeople)))
  'DetectPeople-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectPeople)))
  "Returns string type for a service object of type '<DetectPeople>"
  "openpose_ros/DetectPeople")