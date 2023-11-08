#ifndef DYNAMIXEL
#define DYNAMIXEL

/***************************定义常量************************/
//错误状态
#define        DY_ERROR            0
#define        DY_RIGHT			1


//define the need ID
#define        DY_ID_ID                 0x03
#define        DY_ID_TORQUEABLE         0x18
#define        DY_ID_LED                0x19
#define        DY_ID_GOAL_POSITION      0x1E
#define        DY_ID_REAL_POSITION      0x24
#define        DY_ID_REAL_SPEED         0x26
#define        DY_ID_REAL_LOAD          0x28
#define        DY_ID_REAL_VOL           0x2A
#define        DY_ID_REAL_CURRENT       0x44

//define the need state
#define        DY_ON_TORQUE     1
#define        DY_OFF_TORQUE     0

#define        DY_ON_LED     1
#define        DY_OFF_LED    0


#define			DY_K           (2048/180)
#define			DY_ZERO        2048

/*************定义计算数据***************************/
extern char DY_BaseSendData[223];
extern char DY_ALLState[73];

/*************************************************函数申明****************************************/
int  DY_BaseSend(char Packet_Size, char pID, char CMD);
void DY_READ(char pID, char address, char length);
void DY_WRITE(char pID, char address, char length, char* data);
void DY_REG_WRITE(char pID, char address, char length, char* data);
void DY_ACTION(char pID);
void DY_SYNC_WRITE(int n,char* pID, char address, char length, char* data);
void DY_BULK_READ(int n,char* pID, char* address, char* length);

void DY_ClearError(char pID);
void DY_SetTorque(char pID, char state);
void DY_SetLED(char pID, char state);
void DY_SetPosition(char pID,int data);


void DY_FineError(char pID);
void DY_FindTorque(char pID);
void DY_FindLED(char pID);
void DY_FindPosition(char pID);


int DY_Analysis(char adress ,char *DY_BaseRecData);




#endif
