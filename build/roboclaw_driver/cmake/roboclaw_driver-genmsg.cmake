# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roboclaw_driver: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iroboclaw_driver:/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roboclaw_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_custom_target(_roboclaw_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboclaw_driver" "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_custom_target(_roboclaw_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboclaw_driver" "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboclaw_driver
)
_generate_msg_cpp(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboclaw_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(roboclaw_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboclaw_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roboclaw_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roboclaw_driver_generate_messages roboclaw_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_cpp _roboclaw_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_cpp _roboclaw_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboclaw_driver_gencpp)
add_dependencies(roboclaw_driver_gencpp roboclaw_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboclaw_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roboclaw_driver
)
_generate_msg_eus(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roboclaw_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(roboclaw_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roboclaw_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(roboclaw_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(roboclaw_driver_generate_messages roboclaw_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_eus _roboclaw_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_eus _roboclaw_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboclaw_driver_geneus)
add_dependencies(roboclaw_driver_geneus roboclaw_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboclaw_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboclaw_driver
)
_generate_msg_lisp(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboclaw_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(roboclaw_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboclaw_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roboclaw_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roboclaw_driver_generate_messages roboclaw_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_lisp _roboclaw_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_lisp _roboclaw_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboclaw_driver_genlisp)
add_dependencies(roboclaw_driver_genlisp roboclaw_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboclaw_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roboclaw_driver
)
_generate_msg_nodejs(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roboclaw_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(roboclaw_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roboclaw_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(roboclaw_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(roboclaw_driver_generate_messages roboclaw_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_nodejs _roboclaw_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_nodejs _roboclaw_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboclaw_driver_gennodejs)
add_dependencies(roboclaw_driver_gennodejs roboclaw_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboclaw_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver
)
_generate_msg_py(roboclaw_driver
  "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver
)

### Generating Services

### Generating Module File
_generate_module_py(roboclaw_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roboclaw_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roboclaw_driver_generate_messages roboclaw_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/Stats.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_py _roboclaw_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joel/encoder_publisher_ws/src/roboclaw_driver/msg/SpeedCommand.msg" NAME_WE)
add_dependencies(roboclaw_driver_generate_messages_py _roboclaw_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboclaw_driver_genpy)
add_dependencies(roboclaw_driver_genpy roboclaw_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboclaw_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboclaw_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(roboclaw_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roboclaw_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(roboclaw_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboclaw_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(roboclaw_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roboclaw_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(roboclaw_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboclaw_driver/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(roboclaw_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
