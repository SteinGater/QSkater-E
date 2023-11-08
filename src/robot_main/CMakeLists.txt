cmake_minimum_required(VERSION 2.8.3)
project(robot_main)


set(CMAKE_BUILD_TYPE "Release")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation  
	roscpp
  	rospy
  	std_msgs
	robot_msgl
 
)

#find_package(PkgConfig)
#pkg_check_modules(EIGEN3 REQUIRED eigen3)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

SET(my_HEADERS
	src/lwa_robot/Force_Quadprog.h
        src/lwa_robot/Eigen_Nonlinear_Equ.h
	src/lwa_robot/CJW_KalmanFilter.h
        src/lwa_robot/CJW_Math.h
        src/lwa_robot/CJW_RigidBody.h
        src/lwa_robot/CJW_BranchBased.h
	src/lwa_robot/CJW_BranchLegType.h
	src/lwa_robot/CJW_BranchWheelType.h
	src/lwa_robot/CJW_BranchArmType.h
	src/lwa_robot/CJW_LWABranch.h
        src/lwa_robot/CJW_LWARobot.h
	src/lwa_robot/CJW_GaitBased.h
	src/lwa_robot/CJW_Gait_Bip.h
	src/lwa_robot/CJW_Gait_Qua.h
	src/lwa_robot/CJW_Gait_Hex.h
	src/lwa_robot/CJW_GaitToRobotBased.h

	src/UserRobotStructure.h
	src/Motor_Driver.h
        src/CJW_FootMeasure.h
        src/Servo_DYNAMIXEL.h
        src/Wheel_Driver.cpp
        src/Joint_Manage.h
)
SET(my_SOURCES
	src/lwa_robot/Force_Quadprog.cpp
        src/lwa_robot/Eigen_Nonlinear_Equ.cpp
	src/lwa_robot/CJW_KalmanFilter.cpp
	src/lwa_robot/CJW_Math.cpp
        src/lwa_robot/CJW_RigidBody.cpp
	src/lwa_robot/CJW_BranchBased.cpp
	src/lwa_robot/CJW_BranchLegType.cpp
	src/lwa_robot/CJW_BranchWheelType.cpp
	src/lwa_robot/CJW_BranchArmType.cpp
	src/lwa_robot/CJW_LWABranch.cpp
        src/lwa_robot/CJW_LWARobot.cpp
	src/lwa_robot/CJW_GaitBased.cpp
	src/lwa_robot/CJW_Gait_Bip.cpp
	src/lwa_robot/CJW_Gait_Qua.cpp
	src/lwa_robot/CJW_Gait_Hex.cpp
	src/lwa_robot/CJW_GaitToRobotBased.cpp

        src/UserRobotStructure.cpp
        src/Motor_Driver.cpp
        src/CJW_FootMeasure.cpp
        src/Servo_DYNAMIXEL.cpp
        src/Wheel_Driver.cpp
        src/Joint_Manage.cpp

   	src/main.cpp
)

SET(my_all_SOURCES
    ${my_SOURCES}
    ${my_HEADERS}

)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#    MotorStruct.msg
##    TipStruct.msg
#   RobotStruct.msg
#
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   RobotCommand.srv
#   MotorControl.srv
#   ComputeControl.srv
 #)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES 
#		std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_main
  CATKIN_DEPENDS 
	message_runtime 
	roscpp 
	rospy 
	std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  qpOASES-3.2.1/include/
  unitree_motor_ctrl/include/
)

link_directories(
  ${catkin_LIB_DIRS}
  unitree_motor_ctrl/lib
)

## Declare a C++ library
# add_library(robot_main
#   src/${PROJECT_NAME}/robot_main.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_executable(robot_main ${my_all_SOURCES})

add_dependencies(robot_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(controler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########################################compile qpOASES libraries
FILE(GLOB SRC qpOASES-3.2.1/src/*.cpp)

# library
ADD_LIBRARY(qpOASES STATIC ${SRC})
INSTALL(TARGETS qpOASES
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
SET_TARGET_PROPERTIES(
    qpOASES
    PROPERTIES
    SOVERSION "3.2"
    )

# headers
INSTALL(FILES qpOASES-3.2.1/include/qpOASES.hpp
  DESTINATION include)
INSTALL(DIRECTORY qpOASES-3.2.1/include/qpOASES
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*.ipp"
  PATTERN ".svn" EXCLUDE)
########################################################################

## Specify libraries to link a library or executable target against
 target_link_libraries(robot_main
   qpOASES
   ${catkin_LIBRARIES}
   Unitree_motor_SDK_Linux64_EPOLL.so
 )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS robot_main robot_main_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
