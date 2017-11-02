# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "openpose_ros: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iopenpose_ros:/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genjava REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(openpose_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_custom_target(_openpose_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros" "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" "openpose_ros/PersonDetection:openpose_ros/BodyPartDetection:geometry_msgs/Point"
)

get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_custom_target(_openpose_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros" "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_custom_target(_openpose_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros" "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" "openpose_ros/BodyPartDetection:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genjava;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros
)
_generate_msg_cpp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_cpp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_cpp(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(openpose_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_cpp _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_cpp _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_cpp _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_gencpp)
add_dependencies(openpose_ros_gencpp openpose_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros
)
_generate_msg_eus(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_eus(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_eus(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(openpose_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_eus _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_eus _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_eus _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_geneus)
add_dependencies(openpose_ros_geneus openpose_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_eus)

### Section generating for lang: genjava
### Generating Messages
_generate_msg_java(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros
)
_generate_msg_java(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_java(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_java(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_java}"
)

add_custom_target(openpose_ros_generate_messages_java
  DEPENDS ${ALL_GEN_OUTPUT_FILES_java}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_java)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_java _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_java _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_java _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_genjava)
add_dependencies(openpose_ros_genjava openpose_ros_generate_messages_java)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_java)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros
)
_generate_msg_lisp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_lisp(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_lisp(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(openpose_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_lisp _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_lisp _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_lisp _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_genlisp)
add_dependencies(openpose_ros_genlisp openpose_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros
)
_generate_msg_nodejs(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_nodejs(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_nodejs(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(openpose_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_nodejs _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_nodejs _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_nodejs _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_gennodejs)
add_dependencies(openpose_ros_gennodejs openpose_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros
)
_generate_msg_py(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros
)

### Generating Services
_generate_srv_py(openpose_ros
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv"
  "${MSG_I_FLAGS}"
  "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg;/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros
)

### Generating Module File
_generate_module_py(openpose_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(openpose_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(openpose_ros_generate_messages openpose_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/srv/DetectPeople.srv" NAME_WE)
add_dependencies(openpose_ros_generate_messages_py _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/BodyPartDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_py _openpose_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/robocup/catkin_ws_openpose/src/openpose_ros/msg/PersonDetection.msg" NAME_WE)
add_dependencies(openpose_ros_generate_messages_py _openpose_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_genpy)
add_dependencies(openpose_ros_genpy openpose_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(openpose_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(openpose_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(openpose_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(openpose_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genjava_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/openpose_ros
    DESTINATION ${genjava_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_java)
  add_dependencies(openpose_ros_generate_messages_java std_msgs_generate_messages_java)
endif()
if(TARGET geometry_msgs_generate_messages_java)
  add_dependencies(openpose_ros_generate_messages_java geometry_msgs_generate_messages_java)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(openpose_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(openpose_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(openpose_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(openpose_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(openpose_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(openpose_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
