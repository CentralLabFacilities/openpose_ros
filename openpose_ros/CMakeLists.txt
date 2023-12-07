cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros)

#############
## Deps    ##
#############

find_package(catkin REQUIRED
  roscpp
  cv_bridge
  image_transport
  tf
  clf_perception_vision_msgs
  gender_and_age_msgs
  openpose_ros_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenPose REQUIRED)
include_directories(${OpenPose_INCLUDE_DIRS})

find_package(Caffe REQUIRED)
include_directories(${Caffe_INCLUDE_DIRS})

find_package(Boost REQUIRED system filesystem thread python)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Threads REQUIRED)


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()


#############
## Catkin  ##
#############

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openpose_ros
  CATKIN_DEPENDS roscpp cv_bridge package
#  DEPENDS system_lib
)

#############
## Targets ##
#############

add_executable(detect_people_server src/detect_people_server.cpp)

target_link_libraries(detect_people_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenPose_LIBS}
   ${Caffe_LIBS}
   ${Boost_LIBRARY_DIRS}
   Threads::Threads
 )

#############
## Install ##
#############
install(TARGETS detect_people_server
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

