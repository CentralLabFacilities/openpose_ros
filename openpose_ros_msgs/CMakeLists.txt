cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
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
)

add_service_files(
  FILES
   DetectPeople.srv
   GetPersonAttributes.srv
   GetCrowdAttributes.srv
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
