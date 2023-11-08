#!/bin/bash

#in {/etc/rc.local} {#!/bin/sh} must to chang to {#!/bin/bash}
#chmod -R 777 /dev/ttyUSB*
#chmod -R 777 /dev/ttyS*
#use the sh must write {source /home/hexapodrobot/HexapodRobot/StartWifiRos.bash} in 
#in {/etc/rc.local} before exit 0


sleep 10s

#start the ros
#gnome-terminal -x bash -c "

source /opt/ros/melodic/setup.bash
source ~/QuadrupedRobotCOI/devel/setup.bash
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.1
roslaunch ~/QuadrupedRobotCOI/StartLuanch.launch 


#set the bash to start the ros:
#in{home/.bashrc} to write{source /opt/ros/hydro/setup.bash}
#and {source /home/hexapodrobot/HexapodRobot/devel/setup.bash}


#set the wifi and wire use the same the LAN
#if you use the wire to connect the main compute ,you should have the same set
#expect the changing IP  

#before use ros in other compute ,you should input:
#delete the {HexapodRobot/src/CMake.text}
#remake the package {cd HexapodRobot} {catkin_make -j1}
#the source {HexapodRobot/devel/setup.bash}
#export ROS_MASTER_URI=http://10.42.0.1:11311
#export ROS_IP=10.42.0.N

