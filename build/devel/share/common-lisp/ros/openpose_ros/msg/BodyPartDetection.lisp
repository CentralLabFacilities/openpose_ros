; Auto-generated. Do not edit!


(cl:in-package openpose_ros-msg)


;//! \htmlinclude BodyPartDetection.msg.html

(cl:defclass <BodyPartDetection> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass BodyPartDetection (<BodyPartDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyPartDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyPartDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros-msg:<BodyPartDetection> is deprecated: use openpose_ros-msg:BodyPartDetection instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <BodyPartDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:pos-val is deprecated.  Use openpose_ros-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <BodyPartDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros-msg:confidence-val is deprecated.  Use openpose_ros-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyPartDetection>) ostream)
  "Serializes a message object of type '<BodyPartDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyPartDetection>) istream)
  "Deserializes a message object of type '<BodyPartDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyPartDetection>)))
  "Returns string type for a message object of type '<BodyPartDetection>"
  "openpose_ros/BodyPartDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyPartDetection)))
  "Returns string type for a message object of type 'BodyPartDetection"
  "openpose_ros/BodyPartDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyPartDetection>)))
  "Returns md5sum for a message object of type '<BodyPartDetection>"
  "d353c81ba0c136c3c45ae3297a81d8f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyPartDetection)))
  "Returns md5sum for a message object of type 'BodyPartDetection"
  "d353c81ba0c136c3c45ae3297a81d8f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyPartDetection>)))
  "Returns full string definition for message of type '<BodyPartDetection>"
  (cl:format cl:nil "geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyPartDetection)))
  "Returns full string definition for message of type 'BodyPartDetection"
  (cl:format cl:nil "geometry_msgs/Point pos~%float32 confidence~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyPartDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyPartDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyPartDetection
    (cl:cons ':pos (pos msg))
    (cl:cons ':confidence (confidence msg))
))
