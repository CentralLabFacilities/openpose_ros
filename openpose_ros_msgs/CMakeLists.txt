cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    actionlib
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
  FILES
   GetCrowdAttributesWithPose.action
)

generate_messages(
   DEPENDENCIES
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
