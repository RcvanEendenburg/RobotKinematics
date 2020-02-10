# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "al5d: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ial5d:/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg;-Iactionlib_msgs:/usr/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/usr/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(al5d_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg" "al5d/MoveRobotGoal:al5d/MoveRobotActionGoal:actionlib_msgs/GoalStatus:al5d/MoveRobotActionResult:std_msgs/Header:al5d/MoveRobotResult:actionlib_msgs/GoalID:std_msgs/String:al5d/MoveRobotFeedback:al5d/MoveRobotActionFeedback"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg" "std_msgs/Header:al5d/MoveRobotFeedback:actionlib_msgs/GoalID:std_msgs/String:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg" "std_msgs/Header:actionlib_msgs/GoalID:al5d/MoveRobotResult:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg" ""
)

get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg" NAME_WE)
add_custom_target(_al5d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "al5d" "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg" "std_msgs/Header:al5d/MoveRobotGoal:actionlib_msgs/GoalID"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)
_generate_msg_cpp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
)

### Generating Services

### Generating Module File
_generate_module_cpp(al5d
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(al5d_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(al5d_generate_messages al5d_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_cpp _al5d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(al5d_gencpp)
add_dependencies(al5d_gencpp al5d_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS al5d_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)
_generate_msg_lisp(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
)

### Generating Services

### Generating Module File
_generate_module_lisp(al5d
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(al5d_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(al5d_generate_messages al5d_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_lisp _al5d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(al5d_genlisp)
add_dependencies(al5d_genlisp al5d_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS al5d_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg"
  "${MSG_I_FLAGS}"
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg;/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/usr/share/std_msgs/cmake/../msg/String.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)
_generate_msg_py(al5d
  "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/usr/share/std_msgs/cmake/../msg/Header.msg;/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg;/usr/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
)

### Generating Services

### Generating Module File
_generate_module_py(al5d
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(al5d_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(al5d_generate_messages al5d_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotAction.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotFeedback.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionResult.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/derk/workspace/src/cmake-build-debug/devel/share/al5d/msg/MoveRobotActionGoal.msg" NAME_WE)
add_dependencies(al5d_generate_messages_py _al5d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(al5d_genpy)
add_dependencies(al5d_genpy al5d_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS al5d_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/al5d
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(al5d_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(al5d_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/al5d
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(al5d_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(al5d_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/al5d
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(al5d_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(al5d_generate_messages_py std_msgs_generate_messages_py)
endif()
