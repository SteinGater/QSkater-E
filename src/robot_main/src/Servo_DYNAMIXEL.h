#ifndef SERVO_DYNAMIXEL
#define SERVO_DYNAMIXEL

#include "UserRobotStructure.h"
/*************************************************串口通信设计****************************************/
/***************************定义变量和常量************************/
#define DYNAMIXEL_MAX_BUFF   1000


/************************Joint for robot********************************************/
extern robot_msgl::MotorStruct UserServoExp[User_MainBranchN];
extern robot_msgl::MotorStruct UserServoReal[User_MainBranchN];
extern char User_ServoIDMap[User_MainBranchN];

/*************************************************函数定义****************************************/
int Dynamixel_Init(const char *portName , int baud);
int Dynamixel_Read(int fd, unsigned char* buff,int len);
int Dynamixel_Write(int fd, unsigned char* data, int len);
void Dynamixel_ReadData(void);












/*************************************************DYNAMIXEL数据流****************************************/
/***************************定义常量************************/
//错误状态
#define        DY_ERROR            0
#define        DY_RIGHT			   1

//define the need ID
#define        DY_ID_ID                 7
#define        DY_ID_TORQUEABLE         64
#define        DY_ID_LED                65
#define        DY_ID_RETURN_LEVEL       68
#define        DY_ID_GOAL_POSITION      116
#define        DY_ID_REAL_POSITION      132
#define        DY_ID_REAL_SPEED         128
#define        DY_ID_REAL_LOAD          126
#define        DY_ID_REAL_VOL           144
#define        DY_ID_REAL_CURRENT       126

//define the need state
#define        DY_ON_TORQUE     1
#define        DY_OFF_TORQUE     0

#define        DY_ON_LED     1
#define        DY_OFF_LED    0

#define			DY_K           (2048/M_PI)
#define			DY_ZERO        2048

//define the read feature
#define        DY_READ_ADRESS     DY_ID_REAL_POSITION
#define        DY_READ_LENGTH     4     

/*************定义计算数据***************************/
extern unsigned char DY_BaseSendData[223];
extern unsigned char DY_ALLState[146];

/*************************************************函数定义****************************************/
//send the data based on the data
int  DY_BaseSend(char Packet_Size, char pID, char CMD);
void DY_READ(char pID, unsigned char address, char length);
void DY_WRITE(char pID, unsigned char address, char length, char* data);
void DY_REG_WRITE(char pID, unsigned char address, char length, char* data);
void DY_ACTION(char pID);
void DY_SYNC_WRITE(int n,char* pID, unsigned char address, char length, char* data);
void DY_BULK_READ(int n,char* pID, unsigned char* address, char* length);
//set the data based on the function
void DY_SetTorqueState(char pID, char state);
void DY_SetLED(char pID, char state);
void DY_SetPositionValue(char pID,double angle);
void DY_SetStatusReturnLevel(char pID, char level);
//find the data based on the function
void DY_FindTorqueState(char pID);
void DY_FindLED(char pID);
void DY_FindPosition(char pID);
void DY_FindStatusReturnLevel(char pID);
//analysis the data
int DY_Analysis(unsigned char adress ,unsigned char *DY_BaseRecData);
/*******************Read and Analysis the DY data*************************/
int DY_read_all(int* ID,double* pos);




#endif
