# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "world: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iworld:/home/derk/workspace/src/world/msg;-Istd_msgs:/usr/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(world_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/derk/workspace/src/world/msg/Shape.msg" NAME_WE)
add_custom_target(_world_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "world" "/home/derk/workspace/src/world/msg/Shape.msg" "world/Point2d"
)

get_filename_component(_filename "/home/derk/workspace/src/world/msg/Point2d.msg" NAME_WE)
add_custom_target(_world_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "world" "/home/derk/workspace/src/world/msg/Point2d.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/world/srv/ShapeFinderService.srv" NAME_WE)
add_custom_target(_world_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "world" "/home/derk/workspace/src/world/srv/ShapeFinderService.srv" "world/Point2d:world/Shape"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(world
  "/home/derk/workspace/src/world/msg/Shape.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world
)
_generate_msg_cpp(world
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world
)

### Generating Services
_generate_srv_cpp(world
  "/home/derk/workspace/src/world/srv/ShapeFinderService.srv"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg;/home/derk/workspace/src/world/msg/Shape.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world
)

### Generating Module File
_generate_module_cpp(world
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(world_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(world_generate_messages world_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Shape.msg" NAME_WE)
add_dependencies(world_generate_messages_cpp _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Point2d.msg" NAME_WE)
add_dependencies(world_generate_messages_cpp _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/srv/ShapeFinderService.srv" NAME_WE)
add_dependencies(world_generate_messages_cpp _world_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(world_gencpp)
add_dependencies(world_gencpp world_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS world_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(world
  "/home/derk/workspace/src/world/msg/Shape.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world
)
_generate_msg_lisp(world
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world
)

### Generating Services
_generate_srv_lisp(world
  "/home/derk/workspace/src/world/srv/ShapeFinderService.srv"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg;/home/derk/workspace/src/world/msg/Shape.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world
)

### Generating Module File
_generate_module_lisp(world
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(world_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(world_generate_messages world_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Shape.msg" NAME_WE)
add_dependencies(world_generate_messages_lisp _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Point2d.msg" NAME_WE)
add_dependencies(world_generate_messages_lisp _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/srv/ShapeFinderService.srv" NAME_WE)
add_dependencies(world_generate_messages_lisp _world_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(world_genlisp)
add_dependencies(world_genlisp world_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS world_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(world
  "/home/derk/workspace/src/world/msg/Shape.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world
)
_generate_msg_py(world
  "/home/derk/workspace/src/world/msg/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world
)

### Generating Services
_generate_srv_py(world
  "/home/derk/workspace/src/world/srv/ShapeFinderService.srv"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/world/msg/Point2d.msg;/home/derk/workspace/src/world/msg/Shape.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world
)

### Generating Module File
_generate_module_py(world
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(world_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(world_generate_messages world_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Shape.msg" NAME_WE)
add_dependencies(world_generate_messages_py _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/msg/Point2d.msg" NAME_WE)
add_dependencies(world_generate_messages_py _world_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/world/srv/ShapeFinderService.srv" NAME_WE)
add_dependencies(world_generate_messages_py _world_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(world_genpy)
add_dependencies(world_genpy world_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS world_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/world
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(world_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/world
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(world_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/world
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(world_generate_messages_py std_msgs_generate_messages_py)
endif()
