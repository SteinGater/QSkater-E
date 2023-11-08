#ifndef WHEEL_DRIVER
#define WHEEL_DRIVER

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include "UserRobotStructure.h"

/*******************************parameter*********************************************/
/************************Joint for robot********************************************/
extern robot_msgl::MotorStruct UserWheelExp[User_MainBranchN];
extern robot_msgl::MotorStruct UserWheelReal[User_MainBranchN];
extern char User_WheelIDMap[User_MainBranchN];

/*****************************function***********************************************/



#endif



