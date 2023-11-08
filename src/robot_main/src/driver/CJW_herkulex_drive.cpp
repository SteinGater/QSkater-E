#include "CJW_herkulex_drive.h"
#include "CJW_Serial.h"

//#define NO_SERIAL_SERVO

/*********************定义常量******************************/
#define  DATA_N   7


/*************定义计算数据***************************/
char Herkulex_BaseSendData[223] = { 0 };
char Herkulex_RAMState[73] = { 0 };
char Herkulex_EEPState[54] = { 0 };


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


/************************************************定义功能函数***************************************/

/********************************************基础功能**************************************/
/*******************基础发送功能*************************/
int Herkulex_BaseSend(char Packet_Size, char pID, char CMD)
{
    char SUM = Packet_Size^pID^CMD;

    Herkulex_BaseSendData[0] = (char)0xFF;
    Herkulex_BaseSendData[1] = (char)0xFF;
    Herkulex_BaseSendData[2] = Packet_Size;
    Herkulex_BaseSendData[3] = pID;
    Herkulex_BaseSendData[4] = CMD;

    for (int i = 0; i < (Packet_Size - DATA_N);i++)
    {
        SUM = SUM^(Herkulex_BaseSendData[i+ DATA_N]);
    }

    Herkulex_BaseSendData[5] = SUM & 0xFE;
    Herkulex_BaseSendData[6] = (~SUM) & 0xFE;

    serialWrite(fd_servo1,(unsigned char*)Herkulex_BaseSendData, Packet_Size);

    return 1;
}

/********************EEP_WRITE*****************************/
void Herkulex_EEP_WRITE(char pID, char address, char length, char* data)
{
    Herkulex_BaseSendData[DATA_N] = address;
    Herkulex_BaseSendData[DATA_N + 1] = length;
    for (int i = 0; i < length; i++)
    {
        Herkulex_BaseSendData[DATA_N + 2 + i] = data[i];
    }
    Herkulex_BaseSend((9 + length), pID, 1);
}

/********************EEP_READ*****************************/
void Herkulex_EEP_READ(char pID, char address, char length)
{
    Herkulex_BaseSendData[DATA_N] = address;
    Herkulex_BaseSendData[DATA_N+1] = length;
    Herkulex_BaseSend(9, pID, 2);
}

/********************RAM_WRITE*****************************/
void Herkulex_RAM_WRITE(char pID, char address, char length, char* data)
{
    Herkulex_BaseSendData[DATA_N] = address;
    Herkulex_BaseSendData[DATA_N + 1] = length;
    for (int i = 0; i < length; i++)
    {
        Herkulex_BaseSendData[DATA_N + 2 + i] = data[i];
    }
    Herkulex_BaseSend((9 + length), pID, 3);
}

/********************RAM_READ*****************************/
void Herkulex_RAM_READ(char pID, char address, char length)
{
    Herkulex_BaseSendData[DATA_N] = address;
    Herkulex_BaseSendData[DATA_N + 1] = length;
    Herkulex_BaseSend(9, pID, 4);
}

/********************I_JOG*****************************/
void Herkulex_I_JOG(char pID, char set, char playtime, int JOG)
{
    Herkulex_BaseSendData[DATA_N] = JOG & 0xFF;
    Herkulex_BaseSendData[DATA_N + 1] = (JOG>>8) & 0xFF;
    Herkulex_BaseSendData[DATA_N + 2] = set;
    Herkulex_BaseSendData[DATA_N + 3] = pID;
    Herkulex_BaseSendData[DATA_N + 4] = playtime;
    Herkulex_BaseSend(12, pID, 5);
}

/********************S_JOG*****************************/
void Herkulex_S_JOG(char pID, char set, char playtime, int JOG)
{
    Herkulex_BaseSendData[DATA_N] = playtime;
    Herkulex_BaseSendData[DATA_N + 1] = JOG & 0xFF;
    Herkulex_BaseSendData[DATA_N + 2] = (JOG >> 8) & 0xFF;
    Herkulex_BaseSendData[DATA_N + 3] = set;
    Herkulex_BaseSendData[DATA_N + 4] = pID;
    Herkulex_BaseSend(12, pID, 6);
}
void Herkulex_SS_JOG(char number,char playtime,char* pID, char* set,  int* JOG)
{
    Herkulex_BaseSendData[DATA_N] = playtime;
    for(int i=0;i<number;i++)
    {
        int j=4*i;
        Herkulex_BaseSendData[DATA_N + 1 +j] = JOG[i] & 0xFF;
        Herkulex_BaseSendData[DATA_N + 2 +j] = (JOG[i] >> 8) & 0xFF;
        Herkulex_BaseSendData[DATA_N + 3 +j] = set[i];
        Herkulex_BaseSendData[DATA_N + 4 +j] = pID[i];
    }
    Herkulex_BaseSend((DATA_N+4*number+1),0xFE, 6);
}

/********************STAT*****************************/
void Herkulex_STAT(char pID)
{
    Herkulex_BaseSendData[DATA_N] = 0x00;
    Herkulex_BaseSendData[DATA_N + 1] = 0x40;
    Herkulex_BaseSend(8, pID, 7);
}


/********************************************拓展功能**************************************/
/***********************清楚错误*********************/
void Herkulex_ClearError(char pID)
{
    char clear[2] = { 0x00,0x00 };
    Herkulex_RAM_WRITE(pID, RAM_STATUS, 0x02, clear);
}

/***********************设置灯的颜色*****************/
void Herkulex_SetLED(char pID, char state)
{
    char temp[1] = { state };
    Herkulex_RAM_WRITE(pID, RAM_LED_CONTROL, 0x01, temp);
}

/************************加载力矩*********************/
void Herkulex_SetTorque(char pID, char state)
{
    char temp[1] = { state };
    Herkulex_RAM_WRITE(pID, RAM_TORQUE_CONTROL, 0x01, temp);
}


/************************查询错误状态****************/
void Herkulex_FineError(char pID)
{
    Herkulex_RAM_READ(pID, RAM_STATUS, 0x02);
}

/***************************查询加载情况*************/
void Herkulex_FindTorque(char pID)
{
    Herkulex_RAM_READ(pID, RAM_TORQUE_CONTROL, 0x01);
}

/***************************查询灯情况*************/
void Herkulex_FindLED(char pID)
{
    Herkulex_RAM_READ(pID, RAM_LED_CONTROL, 0x01);
}

/***************************查询位置情况*************/
void Herkulex_FindPosition(char pID)
{
    Herkulex_RAM_READ(pID, RAM_ABSOLUTE_POSITION, 0x02);
}

/********************接受一次处理信息****************************/
int Herkulex_Analysis(char *Herkulex_BaseRecData)
{
    if (Herkulex_BaseRecData[0] == (char)0xFF)
    {
        if (Herkulex_BaseRecData[1] == (char)0xFF)
        {
            char size = Herkulex_BaseRecData[2];
            char ID = Herkulex_BaseRecData[3];
            char SUM = size^ID^Herkulex_BaseRecData[4];
            if (Herkulex_BaseRecData[4] == (char)0x44)//RAM_read响应
            {
                for (int i = 0; i < (size - DATA_N); i++)
                {
                    SUM = SUM^Herkulex_BaseRecData[i + DATA_N];
                }
                if (Herkulex_BaseRecData[5]==(SUM & (char)0xFE))
                {
                    if (Herkulex_BaseRecData[6]==((~SUM) & (char)0xFE))
                    {
                        Herkulex_RAMState[0] = ID;
                        Herkulex_RAMState[4] = size;
                        for (int i = 0; i < (size - DATA_N-4); i++)
                        {
                            Herkulex_RAMState[Herkulex_BaseRecData[7] + i] = Herkulex_BaseRecData[i + DATA_N + 2];
                        }
                        //Herkulex_RAMState[RAM_STATUS] = Herkulex_BaseRecData[size-2];
                        //Herkulex_RAMState[RAM_STATUS + 1] = Herkulex_BaseRecData[size-1];
                        return RIGHT;
                    }
                }
            }
            else if (Herkulex_BaseRecData[4] == 0x42)//EEP_read响应
            {
                for (int i = 0; i < (size - DATA_N); i++)
                {
                    SUM = SUM^Herkulex_BaseRecData[i + DATA_N];
                }
                if (Herkulex_BaseRecData[5] == (SUM & (char)0xFE))
                {
                    if (Herkulex_BaseRecData[6] == ((~SUM) & (char)0xFE))
                    {
                        Herkulex_EEPState[0] = ID;
                        Herkulex_EEPState[4] = size;
                        for (int i = 0; i < (size - DATA_N - 4); i++)
                        {
                            Herkulex_EEPState[Herkulex_BaseRecData[7] + i] = Herkulex_BaseRecData[i + DATA_N + 2];
                        }
                        //Herkulex_EEPState[RAM_STATUS] = Herkulex_BaseRecData[size - 2];
                        //Herkulex_EEPState[RAM_STATUS + 1] = Herkulex_BaseRecData[size - 1];
                        return RIGHT;
                    }
                }
            }
            else if (Herkulex_BaseRecData[4] == 0x47)//STATE响应
            {
                SUM = SUM^Herkulex_BaseRecData[7] ^ Herkulex_BaseRecData[8];
                if ((SUM & 0xFE) == Herkulex_BaseRecData[5])
                {
                    if (((~SUM) & 0xFE) == Herkulex_BaseRecData[6])
                    {
                        Herkulex_RAMState[48] = Herkulex_BaseRecData[7];
                        Herkulex_RAMState[49] = Herkulex_BaseRecData[8];
                        return RIGHT;
                    }
                }
            }
        }
    }
    return ERROR;
}


/*******sxample************
int Analyse_ALL_respond_Her(char *all_inf, int the_number)
{
	char ID = 0;
	int analyse_error = ERROR;
	int readsuccess = ERROR;
	for (int i = 0; i < the_number; i++)
	{
		if (&(all_inf[i]) == 0) return ERROR;
		if ((all_inf[i] == (char)0xFF) && (all_inf[i + 1] == (char)0xFF))//ÕÒµœÊýŸÝ
		{
			analyse_error = Herkulex_Analysis(&(all_inf[i]));
			if (analyse_error == RIGHT)
			{
				ID = Herkulex_RAMState[0];
				ALL_Servo_rel_error[ID] = Herkulex_RAMState[RAM_STATUS];
				ALL_Servo_rel_torque[ID] = Herkulex_RAMState[RAM_TORQUE_CONTROL];
				ALL_Servo_rel_led[ID] = Herkulex_RAMState[RAM_LED_CONTROL];
				double tempangle = (Herkulex_RAMState[RAM_ABSOLUTE_POSITION] & 0x00FF) + (Herkulex_RAMState[RAM_ABSOLUTE_POSITION + 1] & 0xFF) * 256;
				ALL_Servo_rel_angle[ID] = (tempangle * 2 - HERKULEX_ZERO) / HERKULEX_K;
				i = i + Herkulex_RAMState[4] - 1;
				readsuccess = RIGHT;
			}
		}
	}
	return readsuccess;
}
****/
