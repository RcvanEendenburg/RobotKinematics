#!/usr/bin/env bash

#Ros related variables
ROS_WS=~/workspace
LOG_PIPE=rk_log_output

#Config related variables
CONFIG_FOLDER_NAME=config

#Al5d controller
AL5D_NODE_NAME=al5d
AL5D_CONFIG_FILE_NAME_IN=config_sim.ini
AL5D_CONFIG_FILE_NAME_OUT=config_generated.ini
AL5D_SERIAL_PORT_SUBSTITUTE_STR=SERIAL_PORT_SIM
AL5D_NODE_NAME_SUBSTITUTE_STR=NODE_NAME
AL5D_PACKAGE_NAME=al5d

#Robot high level interface
ROBOT_HLI_NODE_NAME=robothli
ROBOT_HLI_CONFIG_FILE_NAME_IN=config_sim.ini
ROBOT_HLI_CONFIG_FILE_NAME_OUT=config_generated.ini
ROBOT_HLI_DRIVER_SUBSTITUTE_STR=LOW_LEVEL_DRIVER
ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR=NODE_NAME
ROBOT_HLI_PACKAGE_NAME=robothli

#TUI
TUI_PACKAGE_NAME=tui
TUI_NODE_NAME=tui

#World
WORLD_PACKAGE_NAME=world
WORLD_NODE_NAME=world_node

#Temporary file
TMP_SERIAL_FILE_NAME=rk_serial_ports

set -e
cleanup() {
    echo "Closing all virtual serial ports..."
    pkill -9 socat
    if [[ -f "/tmp/${TMP_SERIAL_FILE_NAME}" ]]; then
        echo "Deleting temporary serial file..."
        rm "/tmp/${TMP_SERIAL_FILE_NAME}"
    fi
    if [[ -p "/tmp/${LOG_PIPE}" ]]; then
        echo "Deleting log pipe..."
        rm "/tmp/${LOG_PIPE}"
    fi
    pkill -9 ${TUI_NODE_NAME}
    pkill -9 ${AL5D_NODE_NAME}
    pkill -9 ${ROBOT_HLI_NODE_NAME}
    pkill -9 ${WORLD_NODE_NAME}
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
echo "Copying ${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_IN} to ${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}..."
cp ${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_IN} ${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}

#Replacing node name
echo "Replacing ${AL5D_NODE_NAME_SUBSTITUTE_STR} with ${AL5D_NODE_NAME}..."
sed -i "s/$AL5D_NODE_NAME_SUBSTITUTE_STR/$AL5D_NODE_NAME/g" "${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}"

#Replacing serial port
echo "Replacing ${AL5D_SERIAL_PORT_SUBSTITUTE_STR} with ${SERIAL_PORT_B}..."
sed -i "s/$AL5D_SERIAL_PORT_SUBSTITUTE_STR/$SERIAL_PORT_B_SED_STR/g" "${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}"

#Setting robot hli parameters
echo "Copying ${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_IN} to ${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}..."
cp ${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_IN} ${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}

#Replacing node name
echo "Replacing ${ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR} with ${ROBOT_HLI_NODE_NAME}..."
sed -i "s/$ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR/$ROBOT_HLI_NODE_NAME/g" "${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}"

#Replacing low level driver name
echo "Replacing ${ROBOT_HLI_DRIVER_SUBSTITUTE_STR} with ${AL5D_NODE_NAME}..."
sed -i "s/$ROBOT_HLI_DRIVER_SUBSTITUTE_STR/$AL5D_NODE_NAME/g" "${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}"

echo "Creating log pipe..."
mkfifo /tmp/${LOG_PIPE}

echo "Starting ${AL5D_NODE_NAME}..."
rosrun ${AL5D_PACKAGE_NAME} ${AL5D_NODE_NAME} "${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}" | sed -u "s/DEBUG/${AL5D_NODE_NAME}: DEBUG/" | sed -u "s/ERROR/${AL5D_NODE_NAME}: ERROR/" | sed -u "s/WARNING/${AL5D_NODE_NAME}: WARNING/" | sed -u "s/FATAL/${AL5D_NODE_NAME}: FATAL/" > /tmp/${LOG_PIPE} &

echo "Starting ${ROBOT_HLI_NODE_NAME}..."
rosrun ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_NODE_NAME} "${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}" | sed -u "s/DEBUG/${ROBOT_HLI_NODE_NAME}: DEBUG/" | sed -u "s/ERROR/${ROBOT_HLI_NODE_NAME}: ERROR/" | sed -u "s/WARNING/${ROBOT_HLI_NODE_NAME}: WARNING/" | sed -u "s/FATAL/${ROBOT_HLI_NODE_NAME}: FATAL/" > /tmp/${LOG_PIPE} &

echo "Running ${WORLD_NODE_NAME}..."
rosrun ${WORLD_PACKAGE_NAME} ${WORLD_NODE_NAME} &

echo "Running ${TUI_NODE_NAME}..."
rosrun ${TUI_PACKAGE_NAME} ${TUI_NODE_NAME}
