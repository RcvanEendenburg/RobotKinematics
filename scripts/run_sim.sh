#!/usr/bin/env bash

#Ros related variables
ROS_WS=~/workspace
AL5D_NODE_NAME=al5d
ROBOT_HLI_NODE_NAME=robothli
LAUNCH_FILE_NAME=run_sim.launch

#Config related variables
CONFIG_FOLDER_NAME=config

AL5D_CONFIG_FILE_NAME_IN=config_sim.ini
AL5D_CONFIG_FILE_NAME_OUT=config_generated.ini
AL5D_SERIAL_PORT_SUBSTITUTE_STR=SERIAL_PORT_SIM
AL5D_NODE_NAME_SUBSTITUTE_STR=NODE_NAME

ROBOT_HLI_CONFIG_FILE_NAME_IN=config_sim.ini
ROBOT_HLI_CONFIG_FILE_NAME_OUT=config_generated.ini
ROBOT_HLI_DRIVER_SUBSTITUTE_STR=LOW_LEVEL_DRIVER
ROBOT_HLI_NODE_NAME_SUBSTITUTE_STR=NODE_NAME

#Temporary file
TMP_SERIAL_FILE_NAME=rk_serial_ports

echo "Moving to workspace folder..."
cd ${ROS_WS}

echo "Source ROS..."
source devel/setup.bash

echo "Closing all virtual serial ports..."
pkill -9 socat

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

echo "Deleting temporary file..."
rm /tmp/${TMP_SERIAL_FILE_NAME}

echo "Launching ROS launch file..."
roslaunch ${ROBOT_HLI_NODE_NAME} ${LAUNCH_FILE_NAME}