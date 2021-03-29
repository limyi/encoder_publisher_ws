# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "panthera_locomotion: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(panthera_locomotion_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_custom_target(_panthera_locomotion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "panthera_locomotion" "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(panthera_locomotion
  "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panthera_locomotion
)

### Generating Module File
_generate_module_cpp(panthera_locomotion
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panthera_locomotion
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(panthera_locomotion_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(panthera_locomotion_generate_messages panthera_locomotion_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_dependencies(panthera_locomotion_generate_messages_cpp _panthera_locomotion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panthera_locomotion_gencpp)
add_dependencies(panthera_locomotion_gencpp panthera_locomotion_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panthera_locomotion_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(panthera_locomotion
  "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panthera_locomotion
)

### Generating Module File
_generate_module_eus(panthera_locomotion
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panthera_locomotion
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(panthera_locomotion_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(panthera_locomotion_generate_messages panthera_locomotion_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_dependencies(panthera_locomotion_generate_messages_eus _panthera_locomotion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panthera_locomotion_geneus)
add_dependencies(panthera_locomotion_geneus panthera_locomotion_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panthera_locomotion_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(panthera_locomotion
  "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panthera_locomotion
)

### Generating Module File
_generate_module_lisp(panthera_locomotion
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panthera_locomotion
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(panthera_locomotion_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(panthera_locomotion_generate_messages panthera_locomotion_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_dependencies(panthera_locomotion_generate_messages_lisp _panthera_locomotion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panthera_locomotion_genlisp)
add_dependencies(panthera_locomotion_genlisp panthera_locomotion_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panthera_locomotion_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(panthera_locomotion
  "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panthera_locomotion
)

### Generating Module File
_generate_module_nodejs(panthera_locomotion
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panthera_locomotion
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(panthera_locomotion_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(panthera_locomotion_generate_messages panthera_locomotion_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_dependencies(panthera_locomotion_generate_messages_nodejs _panthera_locomotion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panthera_locomotion_gennodejs)
add_dependencies(panthera_locomotion_gennodejs panthera_locomotion_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panthera_locomotion_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(panthera_locomotion
  "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panthera_locomotion
)

### Generating Module File
_generate_module_py(panthera_locomotion
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panthera_locomotion
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(panthera_locomotion_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(panthera_locomotion_generate_messages panthera_locomotion_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sutd/encoder_publisher_ws/src/panthera_locomotion/srv/Status.srv" NAME_WE)
add_dependencies(panthera_locomotion_generate_messages_py _panthera_locomotion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panthera_locomotion_genpy)
add_dependencies(panthera_locomotion_genpy panthera_locomotion_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panthera_locomotion_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panthera_locomotion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panthera_locomotion
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(panthera_locomotion_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(panthera_locomotion_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET std_srvs_generate_messages_cpp)
  add_dependencies(panthera_locomotion_generate_messages_cpp std_srvs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panthera_locomotion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panthera_locomotion
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(panthera_locomotion_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(panthera_locomotion_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET std_srvs_generate_messages_eus)
  add_dependencies(panthera_locomotion_generate_messages_eus std_srvs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panthera_locomotion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panthera_locomotion
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(panthera_locomotion_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(panthera_locomotion_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET std_srvs_generate_messages_lisp)
  add_dependencies(panthera_locomotion_generate_messages_lisp std_srvs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panthera_locomotion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panthera_locomotion
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(panthera_locomotion_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(panthera_locomotion_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET std_srvs_generate_messages_nodejs)
  add_dependencies(panthera_locomotion_generate_messages_nodejs std_srvs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panthera_locomotion)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panthera_locomotion\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panthera_locomotion
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(panthera_locomotion_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(panthera_locomotion_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET std_srvs_generate_messages_py)
  add_dependencies(panthera_locomotion_generate_messages_py std_srvs_generate_messages_py)
endif()
