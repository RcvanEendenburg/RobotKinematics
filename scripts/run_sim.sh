#!/usr/bin/env bash

#Ros related variables
ROS_WS=~/workspace
LOG_PIPE=rk_log_output

#Log mode (not for TUI logging)
#0 = zero logging (log pipe will be written to /dev/null)
#1 = log to file (log pipe will be written to log.txt
#2 = log all (pipe must be emptied with cat)
LOG_MODE=1

#Only if log mode is set to 1
LOG_FILE=rk_log.txt

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

echo "Starting ${AL5D_NODE_NAME}..."
rosrun ${AL5D_PACKAGE_NAME} ${AL5D_NODE_NAME} "${ROS_WS}/src/${AL5D_NODE_NAME}/${CONFIG_FOLDER_NAME}/${AL5D_CONFIG_FILE_NAME_OUT}" |& sed -u -r "s/DEBUG/${AL5D_NODE_NAME}: DEBUG/;s/ERROR/${AL5D_NODE_NAME}: ERROR/;s/WARNING/${AL5D_NODE_NAME}: WARNING/;s/FATAL/${AL5D_NODE_NAME}: FATAL/;s/\[ WARN\] \[[0-9]*.[0-9]*\]:/[${AL5D_NODE_NAME}: ROS WARNING]/;s/\[${AL5D_NODE_NAME}: ERROR\] \[[0-9]*.[0-9]*\]:/[${AL5D_NODE_NAME}: ROS ERROR]/" > /tmp/${LOG_PIPE} &

echo "Starting ${ROBOT_HLI_NODE_NAME}..."
rosrun ${ROBOT_HLI_PACKAGE_NAME} ${ROBOT_HLI_NODE_NAME} "${ROS_WS}/src/${ROBOT_HLI_NODE_NAME}/${CONFIG_FOLDER_NAME}/${ROBOT_HLI_CONFIG_FILE_NAME_OUT}" |& sed -u -r "s/DEBUG/${ROBOT_HLI_NODE_NAME}: DEBUG/;s/ERROR/${ROBOT_HLI_NODE_NAME}: ERROR/;s/WARNING/${ROBOT_HLI_NODE_NAME}: WARNING/;s/FATAL/${ROBOT_HLI_NODE_NAME}: FATAL/;s/\[ WARN\] \[[0-9]*.[0-9]*\]:/[${ROBOT_HLI_NODE_NAME}: ROS WARNING]/;s/\[${ROBOT_HLI_NODE_NAME}: ERROR\] \[[0-9]*.[0-9]*\]:/[${ROBOT_HLI_NODE_NAME}: ROS ERROR]/" > /tmp/${LOG_PIPE} &

echo "Running ${WORLD_NODE_NAME}..."
rosrun ${WORLD_PACKAGE_NAME} ${WORLD_NODE_NAME} |& sed -u -r "s/DEBUG/${WORLD_NODE_NAME}: DEBUG/;s/ERROR/${WORLD_NODE_NAME}: ERROR/;s/FATAL/${WORLD_NODE_NAME}: FATAL/;s/\[ WARN\] \[[0-9]*.[0-9]*\]:/[${WORLD_NODE_NAME}: ROS WARNING]/;s/\[${WORLD_NODE_NAME}: ERROR\] \[[0-9]*.[0-9]*\]:/[${WORLD_NODE_NAME}: ROS ERROR]/" > /tmp/${LOG_PIPE} &

echo "Running ${TUI_NODE_NAME}..."
rosrun ${TUI_PACKAGE_NAME} ${TUI_NODE_NAME}
