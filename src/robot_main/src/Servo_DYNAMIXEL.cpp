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

#include "Servo_DYNAMIXEL.h"
/*************************************************串口通信设计****************************************/
/***************************定义变量和常量************************/
int Dynamixel_fd;
/************************Joint for robot********************************************/
robot_msgl::MotorStruct UserServoExp[User_MainBranchN];
robot_msgl::MotorStruct UserServoReal[User_MainBranchN];
char User_ServoIDMap[User_MainBranchN]={1,2,3,4};
int  User_ServoDir[User_MainBranchN]={-1,-1,-1,-1};

/*************************************************函数定义****************************************/
int Dynamixel_Init(const char *portName , int baud)
{
    int error=ERROR;
    struct termios options;
    int fd;

    if (*portName == '/')       // linux serial port names always begin with /dev
    {
        printf("Opening serial port %s\n", portName);
        fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            perror("init(): Unable to open serial port - ");
        }
        else
        {
            // buffer if there isn't anything there.
            fcntl(fd, F_SETFL, FNDELAY);
            // Configure port for 8N1 transmission, 115200 baud, SW flow control.
            tcgetattr(fd, &options);
            cfsetispeed(&options, baud);
            cfsetospeed(&options, baud);
            options.c_cflag |= (CLOCAL | CREAD);
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            options.c_cflag &= ~CRTSCTS;
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_iflag &= ~(ICRNL| IXON | IXOFF | IXANY);
            options.c_oflag &= ~OPOST;
            // Set the new options for the port "NOW"
            tcsetattr(fd, TCSANOW, &options);
            //得到fd
            Dynamixel_fd=fd;
            error=RIGHT;
            //设置相应模式
            DY_SetStatusReturnLevel(254,1);
        }
    }
    else
    {
        printf("error port name!");
    }
    return error;
}

int Dynamixel_Read(int fd, unsigned char* buff,int len)
{
    unsigned char c;
    unsigned int i;
    int rv;
    rv = read(fd, buff, len);

    if (rv < 0)
    {
        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
            perror("elCommRead() error:");
    }
    return rv;
}

int Dynamixel_Write(int fd, unsigned char* data, int len)
{
  int rv;
  int length = len;
  int totalsent = 0;
  while (totalsent < length)
  {
    rv = write(fd, data + totalsent, length);
    /*if (rv < 0)
      printf("write(): error writing - trying again\n");
    else*/
      totalsent += rv;
  }
  return rv;
}
void Dynamixel_ReadData(void)
{
    int ID[10]={0};
    double angle[10]={0};
    int readn=DY_read_all(ID,angle);
    for(int ii=0;ii<readn;ii++)
    {
        //UserServoReal[ID[ii]-1].ID=ID[ii];
        UserServoReal[ID[ii]-1].angle=User_ServoDir[ii]*angle[ii];
    }
}





/*************************************************DYNAMIXEL数据流****************************************/
/*************定义计算数据***************************/
unsigned char DY_BaseSendData[223]={0};
unsigned char DY_ALLState[146]={0};

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

    Dynamixel_Write(Dynamixel_fd,(unsigned char*)DY_BaseSendData, (Packet_Size+4));

    return true;
}

/*******************read data*************************/
void DY_READ(char pID, unsigned char address, char length)
{
    DY_BaseSendData[5]=address;
    DY_BaseSendData[6]=length;
    DY_BaseSend(4,pID,0x02);
}

/*******************now write data*************************/
void DY_WRITE(char pID, unsigned char address, char length, char* data)
{
    DY_BaseSendData[5]=address;
    for(int i=0;i<length;i++)
    {
        DY_BaseSendData[i+6]=data[i];
    }
    DY_BaseSend((length+3),pID,0x03);
}
/*******************wait the action to  write data*************************/
void DY_REG_WRITE(char pID, unsigned char address, char length, char* data)
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
void DY_SYNC_WRITE(int n,char* pID, unsigned char address, char length, char* data)
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
void DY_BULK_READ(int n,char* pID, unsigned char* address, char* length)
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
/*******************SetTorque*************************/
void DY_SetTorqueState(char pID, char state)
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
void DY_SetPositionValue(char pID,double angle)
{
    int data=(int)((User_ServoDir[pID-1]*angle)*DY_K+DY_ZERO);
    char mydata[4]={(char)(data&0xFF),(char)((data>>8)&0xFF),(char)((data>>16)&0xFF),(char)((data>>24)&0xFF)};
    DY_WRITE(pID,DY_ID_GOAL_POSITION,4,mydata);
}
/********************SetStateReturnLevel************************/
void DY_SetStatusReturnLevel(char pID, char level)
{
    char mydata[2]={level,0};
    DY_WRITE(pID,DY_ID_RETURN_LEVEL,1,mydata);
}
/*******************FindTorque*************************/
void DY_FindTorqueState(char pID)
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
    DY_READ(pID,DY_ID_REAL_POSITION,4);
}
/*******************FindStateReturnLevel*************************/
void DY_FindStatusReturnLevel(char pID)
{
    DY_READ(pID,DY_ID_RETURN_LEVEL,1);
}

/*******************Analysis the read data*************************/
int DY_Analysis(unsigned char adress ,unsigned char *DY_BaseRecData)
{
    if (DY_BaseRecData[0]== 0xFF)
    {
        if (DY_BaseRecData[1]== 0xFF)
        {
            if(DY_BaseRecData[4]== 0x00)
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
                        //printf("DY_BaseRecData[%d] is %X\n",(5+j),DY_BaseRecData[5+j]);
                        DY_ALLState[adress+j]=DY_BaseRecData[5+j];
                    }
                    return (DY_L+4);
                }
            }
        }
    }
    return DY_ERROR;
}
/*******************Read and Analysis the DY data*************************/
int DY_read_all(int* ID,double* pos)
{
    int footn=0;
    unsigned char data[DYNAMIXEL_MAX_BUFF];
    int datan=Dynamixel_Read(Dynamixel_fd, data, DYNAMIXEL_MAX_BUFF);
    //anaysis data
	int readsuccess = 0;
    //printf("datan is %d\n",datan);
    if(datan<6)
    {
        return 0;
    }
    //for(int ii=0;ii<10;ii++){printf("%02X+",data[ii]);}
    //printf("\n");
	for (int searchi = 0; searchi < datan;)
	{
		if ((data[searchi] & 0xFF == 0xFF) && (data[searchi + 1] & 0xFF == 0xFF))
		{
			int result = DY_Analysis(DY_READ_ADRESS, &(data[searchi]));
			if (result == 0)
			{
				searchi++;
			}
			else
			{
				ID[readsuccess] = DY_ALLState[DY_ID_ID];
                //printf("ID is %d :",ID[readsuccess]);
				int endAdress = DY_READ_ADRESS + DY_READ_LENGTH;
				if ((DY_READ_ADRESS <= DY_ID_REAL_POSITION) && (DY_ID_REAL_POSITION <= endAdress))
				{
					int tempangle = (DY_ALLState[DY_ID_REAL_POSITION] & 0x000000FF) + 
                                    (DY_ALLState[DY_ID_REAL_POSITION + 1]<<8 &0x0000FF00) +
                                    (DY_ALLState[DY_ID_REAL_POSITION + 2]<<16 &0x00FF0000) +
                                    (DY_ALLState[DY_ID_REAL_POSITION + 3]<<24 &0xFF000000);
					pos[readsuccess] = ((1.0*tempangle - DY_ZERO) / DY_K);
                    //printf("pos is %f\n",pos[readsuccess]);
				}
				/*if ((DY_READ_ADRESS <= DY_ID_TORQUEABLE) && (DY_ID_TORQUEABLE <= endAdress))
				{
					torque[readsuccess] = DY_ALLState[DY_ID_TORQUEABLE];
				}*/
				searchi = searchi + result;
				readsuccess++;
			}
		}
		else
		{
			searchi++;
		}
        if(readsuccess>=10)
        {
            return readsuccess;
        }
	}
	return readsuccess;
}

    