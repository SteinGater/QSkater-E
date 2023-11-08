#ifndef HERKULEX_DRIVE
#define HERKULEX_DRIVE

/***************************定义常量************************/
//错误状态
#define        ERROR            1
#define        RIGHT			0
//力矩加载状态
#define        Torque_ON       0x60
#define        Break_ON        0x40
#define        Torque_Free     0x00
//灯的状态
#define        LED_ALL_OFF     0x00
#define        LED_G_ON        0x01
#define        LED_B_ON       0x02
#define        LED_R_ON       0x04
//电机运动设置
#define        SET_STOP        0x00
#define        SET_MODE_TRUN   0x02
#define        SET_LED_G       0x04
#define        SET_LED_B       0x08
#define        SET_LED_R       0x10
#define        SET_INVALID     0x20
#define        SET_DIS_VOR     0x40
//RAM数据意义
#define        RAM_STATUS      0x30
#define        RAM_TORQUE_CONTROL  0x34
#define        RAM_LED_CONTROL  0x35
#define        RAM_ABSOLUTE_POSITION 0x3C

#define HERKULEX_K    6.14035
#define HERKULEX_ZERO  1024
/*************定义计算数据***************************/
extern char Herkulex_BaseSendData[223];
extern char Herkulex_RAMState[73];
extern char Herkulex_EEPState[54];

/*************************************************函数申明****************************************/
int  Herkulex_BaseSend(char Packet_Size, char pID, char CMD);
void Herkulex_EEP_WRITE(char pID, char address, char length, char* data);
void Herkulex_EEP_READ(char pID, char address, char length);
void Herkulex_RAM_WRITE(char pID, char address, char length, char* data);
void Herkulex_RAM_READ(char pID, char address, char length);
void Herkulex_I_JOG(char pID, char set, char playtime, int JOG);
void Herkulex_S_JOG(char pID, char set, char playtime, int JOG);
void Herkulex_SS_JOG(char number, char playtime, char *pID, char *set, int *JOG);
void Herkulex_STAT(char pID);

void Herkulex_ClearError(char pID);
void Herkulex_SetTorque(char pID, char state);
void Herkulex_SetLED(char pID, char state);

void Herkulex_FineError(char pID);
void Herkulex_FindTorque(char pID);
void Herkulex_FindLED(char pID);
void Herkulex_FindPosition(char pID);

int Herkulex_Analysis(char *Herkulex_BaseRecData);




#endif
