#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER


#include "ros/ros.h"
#include "UserRobotStructure.h"
#include "unitree_motor_ctrl/include/LSerial.h" //串口通信函数
#include "unitree_motor_ctrl/include/motor_ctrl.h" //声明发送数据、接收数据的结构体，以及函数声明

/***********************关节运动指令*************************/
//分支电机个数
#define MOTOR_BRANCHN_N             3
//指令序列
#define MOTORCOMMAND_POSITION       0
#define MOTORCOMMAND_VELOCITY       1
#define MOTORCOMMAND_TORQUE         2
#define MOTORCOMMAND_ENABLE         3
#define MOTORCOMMAND_ACTION         4
#define MOTORCOMMAND_CLEAR          5
//减速比
#define MOTOR_REDUCTION_RATIO       9.0
//奇异角度边界
#define MOTOR_EXCHANGE_ANGLE        0.1

/**************************电机PID************************/
/*extern double User_Motor_Pos_KP;
extern double User_Motor_Pos_KW;
extern double User_Motor_Vel_KP;
extern double User_Motor_Vel_KW;
extern double User_Motor_Tor_KP;
extern double User_Motor_Tor_KW;*/
/***************************摩擦模型***********************/
#define MOTOR_FRICTION_VEL_ZERO     0.005
#define MOTOR_FRICTION_ACC_ZERO     0.1
/***************************定义常量************************/
extern robot_msgl::MotorStruct UserMotorExp[User_MainBranchN][MOTOR_BRANCHN_N];
extern robot_msgl::MotorStruct UserMotorReal[User_MainBranchN][MOTOR_BRANCHN_N];
extern char User_MotorIDMap[User_MainBranchN][MOTOR_BRANCHN_N];
extern double User_MotorZERO[User_MainBranchN][MOTOR_BRANCHN_N];
extern int User_MotorDIR[User_MainBranchN][MOTOR_BRANCHN_N];
extern int User_MotorFunction[User_MainBranchN][MOTOR_BRANCHN_N];
extern int Motor_fdi[User_MainBranchN];
extern int Ret, Ret1,epfd[4];
extern struct epoll_event eventTest[4];
/************************************************主要控制器标定电机初始角度*******************************************************/
const double User_Motor_Calibration[User_MainBranchN][MOTOR_BRANCHN_N]={
    {0.322,0.682,0.021},
    {0.443,0.541,0.247},
    {0.129,0.624,0.011},
    {0.0182,0.352,0.423}
    /*{2.887/9,6.136/9,0.192/9,0},
    {4.0/9,4.899/9,2.061/9,0},
    {1.119/9,5.589/9,0.147/9,0},
    {6.445/9,3.167/9,4.419/9,0}*/
};
#define USER_JOINT_ZERO_1  0
#define USER_JOINT_ZERO_2  1.152//66度
#define USER_JOINT_ZERO_3  2.572//81.5度+66度
const double User_Joint_ZERO[User_MainBranchN][MOTOR_BRANCHN_N]={
    {-USER_JOINT_ZERO_1,USER_JOINT_ZERO_2,-USER_JOINT_ZERO_3},
    {USER_JOINT_ZERO_1,USER_JOINT_ZERO_2,-USER_JOINT_ZERO_3},
    {USER_JOINT_ZERO_1,-USER_JOINT_ZERO_2,USER_JOINT_ZERO_3},
    {-USER_JOINT_ZERO_1,-USER_JOINT_ZERO_2,USER_JOINT_ZERO_3}
};
/***********************************Motor Controller for Driver***********************************/
int Inital_Motor_Connect(char** seriel);
int Motor_SendRec_One(int Func,int branch,int body,int fd_com,int epfd);
int Motor_Joint_Initial_Calibration(void);
void Motor_Joint_Set_PID(int type,double KP,double KW);
/*******************************************Send ALL Motor move***************************************************/
int Motor_SendRec_Func_OneBranch(int branchi,int epfd);
void Motor_SendRec_Func_ALL(void);
void Motor_Set_Func_OneBranch(int branchi,int type);
void Motor_Set_Func_ALL(int type);
void Set_Motor_One_Branchi_Enable(int branchi, int model);
void Set_Motor_ALL_Enable(int model);
/****************************关节摩擦模型*******************************/
double Motor_Get_Friction(int branchi,int bodyi);


#endif
