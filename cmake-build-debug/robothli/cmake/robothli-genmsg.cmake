# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robothli: 14 messages, 0 services")

set(MSG_I_FLAGS "-Irobothli:/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg;-Iactionlib_msgs:/usr/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/usr/share/std_msgs/cmake/../msg;-Igeometry_msgs:/usr/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robothli_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg" "std_msgs/Header:actionlib_msgs/GoalID:robothli/MoveRobotResult:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg" "robothli/MoveRobotFeedback:std_msgs/Header:actionlib_msgs/GoalID:std_msgs/String:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg" "geometry_msgs/Point:std_msgs/Header:robothli/PickUpObjectGoal:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg" "robothli/PickUpObjectActionGoal:std_msgs/Header:actionlib_msgs/GoalStatus:robothli/PickUpObjectActionResult:geometry_msgs/Point:robothli/PickUpObjectFeedback:robothli/PickUpObjectGoal:actionlib_msgs/GoalID:std_msgs/String:robothli/PickUpObjectResult:robothli/PickUpObjectActionFeedback"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg" "std_msgs/Header:robothli/PickUpObjectResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg" "robothli/MoveRobotGoal:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg" "std_msgs/Header:actionlib_msgs/GoalID:std_msgs/String:robothli/PickUpObjectFeedback:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg" NAME_WE)
add_custom_target(_robothli_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robothli" "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg" "robothli/MoveRobotActionFeedback:robothli/MoveRobotGoal:actionlib_msgs/GoalStatus:robothli/MoveRobotActionGoal:robothli/MoveRobotActionResult:std_msgs/Header:actionlib_msgs/GoalID:std_msgs/String:robothli/MoveRobotFeedback:robothli/MoveRobotResult"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg;/usr/share/geometry_msgs/cmake/../msg/Point.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)
_generate_msg_cpp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
)

### Generating Services

### Generating Module File
_generate_module_cpp(robothli
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robothli_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robothli_generate_messages robothli_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_cpp _robothli_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robothli_gencpp)
add_dependencies(robothli_gencpp robothli_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robothli_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg;/usr/share/geometry_msgs/cmake/../msg/Point.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)
_generate_msg_lisp(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
)

### Generating Services

### Generating Module File
_generate_module_lisp(robothli
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robothli_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robothli_generate_messages robothli_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_lisp _robothli_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robothli_genlisp)
add_dependencies(robothli_genlisp robothli_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robothli_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/geometry_msgs/cmake/../msg/Point.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg;/usr/share/geometry_msgs/cmake/../msg/Point.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)
_generate_msg_py(robothli
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
)

### Generating Services

### Generating Module File
_generate_module_py(robothli
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robothli_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robothli_generate_messages robothli_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(robothli_generate_messages_py _robothli_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robothli_genpy)
add_dependencies(robothli_genpy robothli_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robothli_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robothli
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(robothli_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robothli_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robothli_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robothli
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(robothli_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robothli_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robothli_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robothli
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(robothli_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robothli_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robothli_generate_messages_py geometry_msgs_generate_messages_py)
endif()
