//#include "stdafx.h"//Ubuntu下注释
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Elmo_gold_canopen.h"

/*********************定义常量******************************/
int ELMO_OBJECT[ELMO_OBJECT_N][5] = {
    {0x60,0x3F,0x00,UNT16,0x603F00},
    {0x60,0x40,0x00,UNT16,0x604000},
    {0x60,0x41,0x00,UNT16,0x604100},
    {0x60,0x60,0x00,INT8,0x606000},
    {0x60,0x61,0x00,INT8,0x606100},
    {0x60,0x64,0x00,INT32,0x606400},
    {0x60,0x6C,0x00,INT32,0x606C00},
    {0x60,0x71,0x00,INT16,0x607100},
    {0x60,0x77,0x00,INT16,0x607700},
    {0x60,0x78,0x00,INT16,0x607800},
    {0x60,0x7A,0x00,INT32,0x607A00},
    {0x60,0xFD,0x00,UNT32,0x60FD00},
    {0x60,0xFF,0x00,INT32,0x60FF00},
    {0x22,0x05,0x01,INT16,0x220501},
    {0x2F,0x75,0x00,INT16,0x2F7500},
    {0x60,0x5D,0x00,INT16,0x605D00},
    {0x60,0xF2,0x00,UNT16,0x60F200},
    {0x60,0x7F,0x00,UNT32,0x607F00},
    {0x60,0x83,0x00,UNT32,0x608300},
    {0x60,0x84,0x00,UNT32,0x608400},
    {0x60,0xC2,0x02,INT8,0x60C202},
    {0x60,0xB0,0x00,INT32,0x60B000},
    {0x60,0x86,0x00,INT16,0x608600},
    {0x14,0x00,0x01,UNT32,0x140001},
    {0x14,0x00,0x02,UNT32,0x140002},
    {0x16,0x00,0x00,UNT32,0x160000},
    {0x16,0x00,0x01,UNT32,0x160001},
    {0x16,0x00,0x02,UNT32,0x160002},
    {0x16,0x00,0x03,UNT32,0x160003},
    {0x14,0x01,0x01,UNT32,0x140101},
    {0x14,0x01,0x02,UNT32,0x140102},
    {0x16,0x01,0x00,UNT32,0x160100},
    {0x16,0x01,0x01,UNT32,0x160101},
    {0x16,0x01,0x02,UNT32,0x160102},
    {0x16,0x01,0x03,UNT32,0x160103},
    {0x18,0x00,0x01,UNT32,0x180001},
    {0x18,0x00,0x02,UNT32,0x180002},
    {0x1A,0x00,0x00,UNT32,0x1A0000},
    {0x1A,0x00,0x01,UNT32,0x1A0001},
    {0x1A,0x00,0x02,UNT32,0x1A0002},
    {0x1A,0x00,0x03,UNT32,0x1A0003},
    {0x18,0x01,0x01,UNT32,0x180101},
    {0x18,0x01,0x02,UNT32,0x180102},
    {0x1A,0x01,0x00,UNT32,0x1A0100},
    {0x1A,0x01,0x01,UNT32,0x1A0101},
    {0x1A,0x01,0x02,UNT32,0x1A0102},
    {0x1A,0x01,0x03,UNT32,0x1A0103},
    {0x30,0x6A,0x00,UNT32,0x306A00},

{0x16,0x00,0x01,UNT32,0x160001}
};

const int Canfd[4]={0};
/*************定义计算数据***************************/
unsigned int Elmo_COBID = 0x600;


/*************************************************函数申明****************************************/
int Elmo_ValueToData(int value, int type, char* data);
int Elmo_DataToValue(char* data, int type, int* value);
void Elmo_WriteCanDataSDO(char ID, int object, int data, char* Elmo_SendBase);
void Elmo_ReadCanDataSDO(char ID, int object, char* Elmo_SendBase);
void Elmo_SYNC(char* Elmo_SendBase);
void Elmo_NMT(char ID, char object,char* Elmo_SendBase);
void Elmo_WriteCanDataPDO1(char ID, int object, int data,char* Elmo_SendBase);
void Elmo_WriteCanDataPDO2(char ID, int object, int data,char* Elmo_SendBase);
int Elmo_AnalyCanData(int Elmo_RID, char* Elmo_ReceiveBase, int* IDdata);

/************************************************定义功能函数***************************************/

/********************************************基础功能**************************************/
/*******************数据类型转换函数*****************/
int Elmo_ValueToData(int value, int type, char* data)
{
	int error = 0;
    data[0] = (char)(value & 0x000000FF);
	if (type <= INT8)
	{
		data[1] = 0;
	}
	else
	{
        data[1] = (char)((value & 0x0000FF00)>> 8 );
		if (type <= INT16)
		{
			data[2] = 0;
			data[3] = 0;
		}
		else
		{
            data[2] = (char)((value  & 0x00FF0000)>> 16);
            data[3] = (char)((value  & 0xFF000000)>> 24);
		}
	}
	return error;
}
int Elmo_DataToValue(char* data, int type, int* value)
{
	int error = 0;
	if (type == UNT8)
	{
		value[0]= data[0]&0x000000FF;
		error = 0;
	}
	else if (type == INT8)
	{
        if (data[0] & 0x80) { value[0] = (data[0] & 0x000000FF)+0xFFFFFF00;}
		else { value[0] = data[0] & 0x000000FF; }
		error = 0;
	}
	else if (type == UNT16)
	{
        value[0] = (data[0] & 0x000000FF)+ (data[1] << 8 & 0x0000FF00);
		error = 0;
	}
	else if (type == INT16)
	{
        if (data[1] & 0x80) { value[0] = (data[0] & 0x000000FF) + (data[1] << 8 & 0x0000FF00) + 0xFFFF0000; }
        else { value[0] = (data[0] & 0x000000FF) + (data[1] << 8 & 0x0000FF00); }
		error = 0;
	}
	else if (type == UNT32)
	{
		value[0] = (data[0] & 0x000000FF) + (data[1] << 8 & 0x0000FF00) +
			(data[2] << 16 & 0x00FF0000) + (data[3] << 24 & 0xFF000000);
		error = 0;
	}
	else if (type == INT32)
	{
		value[0] = (data[0] & 0x000000FF) + (data[1] << 8 & 0x0000FF00) +
			(data[2] << 16 & 0x00FF0000) + (data[3] << 24 & 0xFF000000);
		error = 0;
	}
	else
	{
		value[0] = 0;
		error=1;
	}
	return error;
}
/*******************基础写入指令功能****************/
void Elmo_WriteCanDataSDO(char ID, int object, int data,char* Elmo_SendBase)
{
	Elmo_COBID = 0x0600 + ID;
    Elmo_SendBase[0] = 0x22;
	Elmo_SendBase[1] = ELMO_OBJECT[object][1];
	Elmo_SendBase[2] = ELMO_OBJECT[object][0];
	Elmo_SendBase[3] = ELMO_OBJECT[object][2];
    Elmo_ValueToData(data, ELMO_OBJECT[object][3], Elmo_SendBase + 4);
}
/*******************基础读取指令功能**************/
void Elmo_ReadCanDataSDO(char ID, int object,char* Elmo_SendBase)
{
	Elmo_COBID = 0x0600 + ID;
	Elmo_SendBase[0] = 0x40;
	Elmo_SendBase[1] = ELMO_OBJECT[object][1];
	Elmo_SendBase[2] = ELMO_OBJECT[object][0];
	Elmo_SendBase[3] = ELMO_OBJECT[object][2];
	Elmo_SendBase[4] = 0;
	Elmo_SendBase[5] = 0;
	Elmo_SendBase[6] = 0;
	Elmo_SendBase[7] = 0;
}

void Elmo_SYNC(char* Elmo_SendBase)
{
    Elmo_COBID = 0x0080;
    Elmo_SendBase[0] = 0x40;
    Elmo_SendBase[1] = 0;
    Elmo_SendBase[2] = 0;
    Elmo_SendBase[3] = 0;
    Elmo_SendBase[4] = 0;
    Elmo_SendBase[5] = 0;
    Elmo_SendBase[6] = 0;
    Elmo_SendBase[7] = 0;
}

void Elmo_NMT(char ID, char object,char* Elmo_SendBase)
{
    Elmo_COBID = 0x000;
    Elmo_SendBase[0] = object;
    Elmo_SendBase[1] = ID;
    Elmo_SendBase[2] = 0;
    Elmo_SendBase[3] = 0;
    Elmo_SendBase[4] = 0;
    Elmo_SendBase[5] = 0;
    Elmo_SendBase[6] = 0;
    Elmo_SendBase[7] = 0;
}

void Elmo_WriteCanDataPDO1(char ID, int object, int data,char* Elmo_SendBase)
{
    Elmo_COBID = 0x200 + ID;
    //Elmo_ValueToData(data,INT32, (Elmo_SendBase));
    //Elmo_ValueToData(object,UNT32, (Elmo_SendBase+4));
    Elmo_ValueToData(data,INT16, (Elmo_SendBase));
    Elmo_ValueToData(object,UNT16, (Elmo_SendBase+2));
    Elmo_ValueToData(0x00000000,UNT32, (Elmo_SendBase+4));
    //printf("data1=%X\tdata2=%X\t\n",Elmo_SendBase[0],Elmo_SendBase[1]);
}
void Elmo_WriteCanDataPDO2(char ID, int object, int data,char* Elmo_SendBase)
{
    Elmo_COBID = 0x300 + ID;
    Elmo_ValueToData(data,INT32, (Elmo_SendBase));
    Elmo_ValueToData(object,UNT32, (Elmo_SendBase+4));
}

/*******************分析(less 4)返回的数据********************/
//IDdata{0]=ID,IDdata[1]=object,IDdata[2]=data
int Elmo_AnalyCanData(int Elmo_RID,char* Elmo_ReceiveBase,int* IDdata)
{
	int error = 0;
	IDdata[0] = (Elmo_RID & 0x7F);
    if((Elmo_RID&0x180)== 0x180)//TPDO1
    {

    }
    else if((Elmo_RID&0x280)== 0x280)//TPDO2
    {

    }
    else if ((Elmo_ReceiveBase[0]&0x40)== 0x40)
	{
		int getobject = (Elmo_ReceiveBase[1] << 8 & 0xFF00) +
			(Elmo_ReceiveBase[2] << 16 & 0xFF0000) +
            (Elmo_ReceiveBase[3]);
        IDdata[1] = getobject; error = 2;
        for(int ob=0;ob<ELMO_OBJECT_N;ob++)
        {
            if (getobject == ELMO_OBJECT[ob][4]) { IDdata[1] = ob; error=0;}
        }
		if(error==0)
		{
			Elmo_DataToValue(Elmo_ReceiveBase + 4, ELMO_OBJECT[IDdata[1]][3], IDdata + 2);
		}
	}
	else
	{
		error = 1;
	}
	return error;
}


