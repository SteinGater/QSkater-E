#ifndef CJW_USERROBOTSTRUCTURE
#define CJW_USERROBOTSTRUCTURE

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include "robot_msgl/MotorStruct.h"
#include "robot_msgl/TipStruct.h"
#include "robot_msgl/RobotStruct.h"
#include "robot_msgl/ControlStruct.h"
#include "robot_msgl/RobotCommand.h"
#include "robot_msgl/MotorControl.h"
#include "robot_msgl/ComputeControl.h"

#include "lwa_robot/CJW_GaitToRobotBased.h"

using namespace std;

/***************************define the type length********************************************/
#define User_MainBranchN        4
#define User_MainBranchBodyN    5
//几何尺寸
#define User_MainBody_X         0.21
#define User_MainBody_Y         0.068
#define User_MainBody_Z         0
#define User_LEG_L1             0.0
#define User_LEG_L2             0.2
#define User_LEG_L3             0.2    //0.23898
#define User_FOOT_Y             0.084
#define User_LEG_WheelR         0.032
//质量参数
#define User_MainBody_Mass      6.253
#define User_LEG_Mass1          0.737
#define User_LEG_Mass2          1.33
#define User_LEG_Mass3          0.4
#define User_LEG_MassD          0.01
#define User_LEG_MassW          0.14
//关节限制参数
#define User_Joint_MaxV         (6*M_PI)
#define User_Joint_MaxA         (100*M_PI)
#define User_Joint_MaxF         40
#define User_Wheel_MaxV         (6*M_PI)
//机身姿态限制参数
#define User_Body_Limit_XY       10000.0
#define User_Body_Limit_ZMAX     0.45
#define User_Body_Limit_W0       (6.0*M_PI/18.0)
//机身状态错误编码
#define ERROR_BODY_ORI_OVER      1
#define ERROR_BODY_X_OVER        2
#define ERROR_BODY_Y_OVER        3
#define ERROR_BODY_Z_OVERMAX     4
#define ERROR_BODY_Z_OVERMIN     5

/***********************默认机身位置和线速度估计参数*****************************************/
#define imu_process_noise_position    0.02//0.02 //IMU姿态模型预测位置的协方差
#define imu_process_noise_velocity    0.02//0.02 //IMU姿态模型预测线速度的协方差
#define foot_process_noise_position   0.002//0.002 //关节运动学模型预测足位置的协方差
#define foot_sensor_noise_position    0.001//0.001 //观测足位置的协方差
#define foot_sensor_noise_velocity    0.1//0.1     //观测足线速度的协方差
#define foot_height_sensor_noise      0.001//0.001 //观测足高度的协方差
#define extern_sensor_noise_position  1000.0      //外部位置传感器的协方差
#define extern_sensor_noise_velocity  1000.0      //外部速度传感器的协方差
extern double Robot_PV_Esti_Para[8];

/********************************************************
the robot structure initial
******************************************************/
extern struct CJW_JointStruct User_MainBody0;
extern struct CJW_TipStruct User_MainTip0[User_MainBranchN];


/*********************************设置机器人控制参数*************************************/
extern double User_BodyFKp0[6]; //动力学机身位姿控制比例系数
extern double User_BodyFKd0[6]; //动力学机身速度控制比例系数
extern double User_BodyVKp[6];
extern double User_Qua_S[6];    //最优化权重
extern double User_Qua_W[3];    //安全性权重
extern double User_InsideKp[3]; //支撑腿内部相对位置控制比例系数
extern double User_InsideKd[3]; //支撑腿内部相对速度控制比例系数
extern double User_SwingKp[3];  //摆动腿位置控制比例系数
extern double User_SwingKd[3];  //摆动腿速度控制比例系数

extern double User_BodyAccMax[6];  //机身最大加速度
extern double User_BodyAccMin[6]; //机身最小加速度

/********************************************************
the robot of user
  ******************************************************/
extern CJW_Math<double> User_Math;
extern CJW_LWARobot User_RobotExp;
extern CJW_LWARobot User_RobotReal;
extern CJW_Gait_Qua User_Gait_Qua;
extern CJW_GaitToRobotBased User_Gait;

/********************************************************
the robot  initial the struct setting
  ******************************************************/
void MyRobot_Init(void);
/***********************监测机身位置姿态是否正常******************************/
int  MyRobot_Check_Body(CJW_LWARobot* robot);
/*******************************public robot state*************************************************/
void MapRobotToPublicState(CJW_LWARobot* in,robot_msgl::RobotStruct *out);
/*******************************msg to CJW struct*************************************************/
void MapTipMsgToJointStruct(robot_msgl::TipStruct *in,struct CJW_JointStruct* out);
void MapTipMsgToTipStruct(robot_msgl::TipStruct *in,struct CJW_TipStruct* out);



#endif
