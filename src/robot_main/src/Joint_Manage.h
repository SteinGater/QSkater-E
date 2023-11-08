#ifndef JOINT_MANAGE
#define JOINT_MANAGE

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

#include "Servo_DYNAMIXEL.h"
#include "Motor_Driver.h"
#include "Wheel_Driver.h"
/*******************************parameter*********************************************/



/*****************************function***********************************************/
/******************************set joint enable********************************/
void Set_Joint_One_Branchi_Enable(int branchi, int model);
void Set_Joint_ALL_Enable(int model);
/********************************transform the joint to the motor to control robot**********************************/
void MapBranchiJointToMotorJoint(CJW_LWARobot* in, int controlmodel);
/********************************transform the real joint to robot**********************************/
void MapMotorJointToBranchiJoint(CJW_LWARobot* in);


#endif



