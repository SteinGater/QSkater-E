#include "CJW_DYNAMIXEL.h"
#include "CJW_serial.h"
#include "stdio.h"

/*********************定义常量******************************/

/*************定义计算数据***************************/
char DY_BaseSendData[223]={0};
char DY_ALLState[73]={0};

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

/************************************************定义功能函数***************************************/

/********************************************基础功能**************************************/
/*******************基础发送功能*************************/
int  DY_BaseSend(char Packet_Size, char pID, char CMD)
{
    DY_BaseSendData[0]=0xFF;
    DY_BaseSendData[1]=0xFF;
    DY_BaseSendData[2]=pID;
    DY_BaseSendData[3]=Packet_Size;
    DY_BaseSendData[4]=CMD;

    int sum=0;
    for(int i=2;i<Packet_Size+3;i++)
    {
        sum=sum+DY_BaseSendData[i];
    }
    DY_BaseSendData[Packet_Size+3]=0xFF-sum&0xFF;

    serialWrite(fd_servo0,(unsigned char*)DY_BaseSendData, (Packet_Size+4));

    return true;
}

/*******************read data*************************/
void DY_READ(char pID, char address, char length)
{
    DY_BaseSendData[5]=address;
    DY_BaseSendData[6]=length;
    DY_BaseSend(4,pID,0x02);
}

/*******************now write data*************************/
void DY_WRITE(char pID, char address, char length, char* data)
{
    DY_BaseSendData[5]=address;
    for(int i=0;i<length;i++)
    {
        DY_BaseSendData[i+6]=data[i];
    }
    DY_BaseSend((length+3),pID,0x03);
}
/*******************wait the action to  write data*************************/
void DY_REG_WRITE(char pID, char address, char length, char* data)
{
    DY_BaseSendData[5]=address;
    for(int i=0;i<length;i++)
    {
        DY_BaseSendData[i+6]=data[i];
    }
    DY_BaseSend((length+3),pID,0x04);
}
/*******************action*************************/
void DY_ACTION(char pID)
{
    DY_BaseSend(2,pID,0x05);
}
/*******************sync write to data*************************/
void DY_SYNC_WRITE(int n,char* pID, char address, char length, char* data)
{
    DY_BaseSendData[5]=address;
    DY_BaseSendData[6]=length;
    for(int i=0;i<n;i++)
    {
        int s1=7+(length+1)*i;
        int s2=i*length;
        DY_BaseSendData[s1]=pID[i];
        for(int j=0;j<length;j++)
        {
            DY_BaseSendData[s1+1+j]=data[s2+j];
        }
    }
    DY_BaseSend(((length+1)*n+4),0xFE,0x83);
}
/*******************bulk read to data*************************/
void DY_BULK_READ(int n,char* pID, char* address, char* length)
{
    DY_BaseSendData[5]=0x00;
    for(int i=0;i<n;i++)
    {
        int s1=3*i+6;
        DY_BaseSendData[s1]=length[i];
        DY_BaseSendData[s1+1]=pID[i];
        DY_BaseSendData[s1+2]=address[i];
    }
    DY_BaseSend((3*n+3),0xFE,0x92);
}



/*******************function*************************************************/
/*******************clear error*************************/
void DY_ClearError(char pID)
{

}
/*******************SetTorque*************************/
void DY_SetTorque(char pID, char state)
{
    char mydata[2]={state,0};
    DY_WRITE(pID,DY_ID_TORQUEABLE,1,mydata);
}
/*******************SetLED*************************/
void DY_SetLED(char pID, char state)
{
    char mydata[2]={state,0};
    DY_WRITE(pID,DY_ID_LED,1,mydata);
}
/*******************SetPosition*************************/
void DY_SetPosition(char pID,int data)
{
    char mydata[2]={data&0xFF,(data>>8)&0xFF};
    DY_WRITE(pID,DY_ID_GOAL_POSITION,2,mydata);
}


/*******************FindError*************************/
void DY_FineError(char pID)
{

}
/*******************FindTorque*************************/
void DY_FindTorque(char pID)
{
    DY_READ(pID,DY_ID_TORQUEABLE,1);
}
/*******************FindLED*************************/
void DY_FindLED(char pID)
{
    DY_READ(pID,DY_ID_LED,1);
}
/*******************FindPosition*************************/
void DY_FindPosition(char pID)
{
    DY_READ(pID,DY_ID_REAL_POSITION,2);
}
/*******************Analysis the read data*************************/
int DY_Analysis(char adress ,char *DY_BaseRecData)
{
    if (DY_BaseRecData[0] == (char)0xFF)
    {
        if (DY_BaseRecData[1] == (char)0xFF)
        {
            if(DY_BaseRecData[4]==0)
            {
                int DY_L=DY_BaseRecData[3];
                int sum=0;
                for(int i=2;i<DY_L+3;i++)
                {
                    sum=sum+DY_BaseRecData[i];
                }
                sum=0xFF-(sum&0xFF);
                //printf("sum is %02X\n",sum);
                if((sum&0xFF)==(DY_BaseRecData[DY_L+3]&0xFF))
                {
                    DY_ALLState[DY_ID_ID]=DY_BaseRecData[2];
                    for(int j=0;j<(DY_L-2);j++)
                    {
                        DY_ALLState[adress+j]=DY_BaseRecData[5+j];
                    }
                    return (DY_L+4);
                }
            }
        }
    }
    return DY_ERROR;
}

/**********example
int Analyse_ALL_respond_DY(char *all_inf, int the_number)
{
	int readsuccess = 0;

	for (int searchi = 0; searchi < the_number;)
	{
		if ((all_inf[searchi] & 0xFF == 0xFF) && (all_inf[searchi + 1] & 0xFF == 0xFF))
		{
			int result = DY_Analysis(DY_READ_ADRESS, &(all_inf[searchi]));
			if (result == 0)
			{
				searchi++;
			}
			else
			{
				int tempid = DY_ALLState[DY_ID_ID];
				int endAdress = DY_READ_ADRESS + DY_READ_LENGTH;
				if ((DY_READ_ADRESS <= DY_ID_REAL_POSITION) && (DY_ID_REAL_POSITION <= endAdress))
				{
					int tempangle = (DY_ALLState[DY_ID_REAL_POSITION] & 0x000000FF) + (DY_ALLState[DY_ID_REAL_POSITION + 1] & 0x000000FF) * 256;
					ALL_Servo_rel_angle[tempid] = ((1.0*tempangle - DY_ZERO) / DY_K + Servo_ZERO[tempid])*Servo_DIR[tempid];
					//printf("id %d servo angle is %.3f\n",tempid,ALL_Servo_rel_angle[tempid]);
				}
				if ((DY_READ_ADRESS <= DY_ID_TORQUEABLE) && (DY_ID_TORQUEABLE <= endAdress))
				{
					ALL_Servo_rel_torque[tempid] = DY_ALLState[DY_ID_TORQUEABLE];
				}
				if ((DY_READ_ADRESS <= DY_ID_LED) && (DY_ID_LED <= endAdress))
				{
					ALL_Servo_rel_led[tempid] = DY_ALLState[DY_ID_LED];
				}
				searchi = searchi + result;
				readsuccess++;
			}
		}
		else
		{
			searchi++;
		}
	}
	return readsuccess;
}
*****/