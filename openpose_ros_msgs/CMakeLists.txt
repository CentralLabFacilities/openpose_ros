cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    actionlib_msgs
    sensor_msgs
    message_generation
    geometry_msgs
    gender_and_age_msgs
)

add_message_files(
  FILES
   BodyPartDetection.msg
   PersonDetection.msg
   PersonAttributes.msg
   PersonAttributesWithPose.msg
   Posture.msg
   Gesture.msg
)

add_service_files(
  FILES
   DetectPeople.srv
   GetPersonAttributes.srv
   GetCrowdAttributes.srv
   GetCrowdAttributesWithPose.srv
   GetFollowRoi.srv
)

add_action_files(
  DIRECTORY action
  FILES
   GetCrowdAttributesWithPose.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
   gender_and_age_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
)
