#!/usr/bin/env bash

#Ros related variables
ROS_WS=/workspace
LOG_PIPE=rk_log_output

#Log mode
#0 = zero logging (log pipe will be written to /dev/null)
#1 = log to file (log pipe will be written to log.txt
#2 = log all (pipe must be emptied with cat)
LOG_MODE=2

#Only if log mode is set to 1
LOG_FILE=rk_log.txt

#Config related variables
CONFIG_FOLDER_NAME=config

#Al5d controller
AL5D_NODE_NAME=al5d
AL5D_CONFIG_FILE_NAME_IN=config.ini
AL5D_CONFIG_FILE_NAME_OUT=config_generated.ini
AL5D_SERIAL_PORT_SUBSTITUTE_STR=SERIAL_PORT_SIM
AL5D_NODE_NAME_SUBSTITUTE_STR=NODE_NAME
AL5D_PACKAGE_NAME=al5d

#Robot high level interface
ROBOT_HLI_NODE_NAME=robothli
ROBOT_HLI_CONFIG_FILE_NAME_IN=config.ini
ROBOT_HLI_CONFIG_FILE_NAME_OUT=config_generated.ini
ROBOT_HLI_DRIVER_SUBSTITUTE_STR=LOW_LEVEL_DRIVER
ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR=NODE_NAME
ROBOT_HLI_PACKAGE_NAME=robothli

#World
WORLD_PACKAGE_NAME=world
WORLD_NODE_NAME=world_node
WORLD_CONFIG_FILE_NAME_IN=config.ini
WORLD_CONFIG_FILE_NAME_OUT=config_generated.ini
WORLD_IMAGE_PATH_SUBSTITUTE_STR=IMAGE_PATH
WORLD_IMAGE_PATH=TestImage/Blocks01.jpg
WORLD_CAMERA_ENABLED_SUBSTITUTE_STR=CAMERA_ENABLED
WORLD_CAMERA_ENABLED=1

#Serial forwarder
SERIAL_FORWARDER_PACKAGE_NAME=serial_forwarder
SERIAL_FORWARDER_NODE_NAME=serial_forwarder

#Temporary file
TMP_SERIAL_FILE_NAME=rk_serial_ports

set -e
cleanup() {
    echo "Closing all virtual serial ports..."
    if [[ -f "/tmp/${TMP_SERIAL_FILE_NAME}" ]]; then
        echo "Deleting temporary serial file..."
        rm "/tmp/${TMP_SERIAL_FILE_NAME}"
    fi
    if [[ -p "/tmp/${LOG_PIPE}" ]]; then
        echo "Deleting log pipe..."
        rm "/tmp/${LOG_PIPE}"
    fi
    ps | sed -e "/bash/d" -e "/awk/d" -e "/ps/d" -e "/sed/d" -e "/grep/d" | awk '{print $1}' | grep -v PID | while read process
    do
        kill ${process}
    done
}

#$1 is to be replaced
#$2 is the replacement
#$3 is the package
#$4 is the config out file
replace_config_value() {
    echo "Replacing value... $1, $2, $3, $4"
    sed -i "s/$1/$2/g" "${ROS_WS}/src/$3/${CONFIG_FOLDER_NAME}/$4"
}

#$1 is the package
#$2 is the config in file
#$3 is the config out file
prepare_generated_ini_file(){
    cp ${ROS_WS}/src/$1/${CONFIG_FOLDER_NAME}/$2 ${ROS_WS}/src/$1/${CONFIG_FOLDER_NAME}/$3
}

#$1 is the package
#$2 is the node name
#$3 is the config out file
#$4 0 is async, 1 is sync
start_ros_node(){
    echo "Starting $2..."
    REPLACE_DEBUG_STR="s/DEBUG/$2: DEBUG/"
    REPLACE_ERROR_STR="s/ERROR/$2: ERROR/"
    REPLACE_WARNING_STR="s/WARNING/$2: WARNING/"
    REPLACE_FATAL_STR="s/FATAL/$2: FATAL/"
    REPLACE_ROS_WARNING_STR="s/\[ WARN\] \[[0-9]*.[0-9]*\]:/[$2: ROS WARNING]/"
    REPLACE_ROS_ERROR_STR="s/\[$2: ERROR\] \[[0-9]*.[0-9]*\]:/[$2: ROS ERROR]/"

    REPLACE_LOG_STR="${REPLACE_DEBUG_STR};${REPLACE_ERROR_STR};${REPLACE_WARNING_STR};${REPLACE_FATAL_STR};${REPLACE_ROS_WARNING_STR};${REPLACE_ROS_ERROR_STR}"

    if [[ ${4} -eq 0 ]]; then
        rosrun $1 $2 "${ROS_WS}/src/$1/${CONFIG_FOLDER_NAME}/$3" |& sed -u -r "${REPLACE_LOG_STR}" > /tmp/${LOG_PIPE} &
    else
        rosrun $1 $2 "${ROS_WS}/src/$1/${CONFIG_FOLDER_NAME}/$3" |& sed -u -r "${REPLACE_LOG_STR}" > /tmp/${LOG_PIPE}
    fi
}

trap cleanup EXIT
echo "Moving to workspace folder..."
cd ${ROS_WS}

echo "Source ROS..."
source devel/setup.bash

echo "Opening virtual serial ports..."
( socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>&1 | grep --line-buffered -Eo "/dev/pts/\w+" > /tmp/${TMP_SERIAL_FILE_NAME} ) &
sleep 1
echo "Serial ports opened! Written information to /tmp/${TMP_SERIAL_FILE_NAME}."

SERIAL_PORT_A=$(head -n 1 /tmp/${TMP_SERIAL_FILE_NAME})
SERIAL_PORT_B=$(tail -n 1 /tmp/${TMP_SERIAL_FILE_NAME})

SERIAL_PORT_A_SED_STR=${SERIAL_PORT_A////\\\/}
SERIAL_PORT_B_SED_STR=${SERIAL_PORT_B////\\\/}

echo "Serial ports ${SERIAL_PORT_A} and ${SERIAL_PORT_B} are opened."

#Setting al5d parameters
prepare_generated_ini_file ${AL5D_PACKAGE_NAME} ${AL5D_CONFIG_FILE_NAME_IN} ${AL5D_CONFIG_FILE_NAME_OUT}
replace_config_value ${AL5D_NODE_NAME_SUBSTITUTE_STR} ${AL5D_NODE_NAME} ${AL5D_PACKAGE_NAME} ${AL5D_CONFIG_FILE_NAME_OUT}
replace_config_value ${AL5D_SERIAL_PORT_SUBSTITUTE_STR} ${SERIAL_PORT_B_SED_STR} ${AL5D_PACKAGE_NAME} ${AL5D_CONFIG_FILE_NAME_OUT}

#Setting robot hli parameters
prepare_generated_ini_file ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_CONFIG_FILE_NAME_IN} ${ROBOT_HLI_CONFIG_FILE_NAME_OUT}
replace_config_value ${ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR} ${ROBOT_HLI_NODE_NAME} ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_CONFIG_FILE_NAME_OUT}
replace_config_value ${ROBOT_HLI_DRIVER_SUBSTITUTE_STR} ${AL5D_NODE_NAME} ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_CONFIG_FILE_NAME_OUT}

#Setting world parameters
WORLD_IMAGE_PATH=${ROS_WS}/src/${WORLD_PACKAGE_NAME}/${WORLD_IMAGE_PATH}
WORLD_IMAGE_PATH=${WORLD_IMAGE_PATH////\\\/}
prepare_generated_ini_file ${WORLD_PACKAGE_NAME} ${WORLD_CONFIG_FILE_NAME_IN} ${WORLD_CONFIG_FILE_NAME_OUT}
replace_config_value ${WORLD_IMAGE_PATH_SUBSTITUTE_STR} ${WORLD_IMAGE_PATH} ${WORLD_PACKAGE_NAME} ${WORLD_CONFIG_FILE_NAME_OUT}
replace_config_value ${WORLD_CAMERA_ENABLED_SUBSTITUTE_STR} ${WORLD_CAMERA_ENABLED} ${WORLD_PACKAGE_NAME} ${WORLD_CONFIG_FILE_NAME_OUT}

echo "Creating log pipe..."
mkfifo /tmp/${LOG_PIPE}

if [[ ${LOG_MODE} -eq 0 ]]; then
    echo "Logging disabled"
    cat /tmp/${LOG_PIPE} > /dev/null &
elif [[ ${LOG_MODE} -eq 1 ]]; then
    echo "Logging to /tmp/${LOG_FILE}"
    cat /tmp/${LOG_PIPE} > /tmp/${LOG_FILE} &
elif [[ ${LOG_MODE} -eq 2 ]]; then
    echo "Logging enabled, make sure to read the pipe in another terminal. E.g. cat /tmp/${LOG_PIPE}"
else
    echo "Log mode is not set to 0, 1 or 2. Exiting..."
    exit 1
fi

rosrun ${SERIAL_FORWARDER_PACKAGE_NAME} ${SERIAL_FORWARDER_NODE_NAME} ${SERIAL_PORT_A} &

start_ros_node ${AL5D_PACKAGE_NAME} ${AL5D_NODE_NAME} ${AL5D_CONFIG_FILE_NAME_OUT} 0
start_ros_node ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_NODE_NAME} ${ROBOT_HLI_CONFIG_FILE_NAME_OUT} 0
start_ros_node ${WORLD_PACKAGE_NAME} ${WORLD_NODE_NAME} ${WORLD_CONFIG_FILE_NAME_OUT} 0

if [[ ${LOG_MODE} -eq 2 ]]; then
    cat /tmp/${LOG_PIPE}
fi