#ifndef __Elmo_gold_canopen_h
#define __Elmo_gold_canopen_h




/***************************定义常量************************/
//object number
#define     ELMO_OBJECT_N					49
//data type
#define     UNT8							0
#define     INT8							1
#define     UNT16							2
#define     INT16							3
#define     UNT32							4
#define     INT32							5
#define     FLOAT							6
//define Elmo canopen object dictionary index(16)+subindex(8)+datatype(4)
#define     CAN_ERROR_CODE					0
#define	  CAN_CONTROL_WORD				1
#define     CAN_STATUS_WORD					2
#define     CAN_MODES_OPERATION				3
#define     CAN_MODES_OPERATION_DISPLAY		4
#define     CAN_POSITION_ACTUAL				5
#define     CAN_VELOCITY_ACTUAL				6
#define     CAN_TARGET_TORQUE				7
#define     CAN_TORQUE_ACTUAL				8
#define     CAN_CURRENT_ACTUAL				9
#define     CAN_TARGET_POSITION				10
#define     CAN_DIGITAL_INPUTS				11
#define	  CAN_TARGET_VELOCITY				12
#define     CAN_ANALOG_INPUTS				13
#define     CAN_EX_TIMEOUT                        14
#define     CAN_HALF_CODE                          15
#define     CAN_POSITION_CODE                  16
#define     CAN_PROFILE_MAX_VELOCITY                    17
#define     CAN_PROFILE_MAX_ACCELERATION           18
#define     CAN_PROFILE_MAX_DECELERATION           19
#define     CAN_COMMU_PERIOD                                  20
#define     CAN_POSITION_OFFSET                                21
#define     CAN_MOTION_TYPE                                        22
#define     CAN_RPOD1_CP1                                             23
#define     CAN_RPOD1_CP2                                            24
#define     CAN_RPOD1_map0                                            25
#define     CAN_RPOD1_map1                                            26
#define     CAN_RPOD1_map2                                            27
#define     CAN_RPOD1_map3                                            28
#define     CAN_RPOD2_CP1                                             29
#define     CAN_RPOD2_CP2                                            30
#define     CAN_RPOD2_map0                                            31
#define     CAN_RPOD2_map1                                            32
#define     CAN_RPOD2_map2                                            33
#define     CAN_RPOD2_map3                                           34
#define     CAN_TPOD1_CP1                                             35
#define     CAN_TPOD1_CP2                                            36
#define     CAN_TPOD1_map0                                            37
#define     CAN_TPOD1_map1                                           38
#define     CAN_TPOD1_map2                                            39
#define     CAN_TPOD1_map3                                            40
#define     CAN_TPOD2_CP1                                             41
#define     CAN_TPOD2_CP2                                            42
#define     CAN_TPOD2_map0                                            43
#define     CAN_TPOD2_map1                                           44
#define     CAN_TPOD2_map2                                            45
#define     CAN_TPOD2_map3                                            46
#define     CAN_EC                                                                 47



#define   CAN_TEST                                                        ELMO_OBJECT_N-1
//define the directionary
extern int ELMO_OBJECT[ELMO_OBJECT_N][5];




/***************************定义变量************************/
extern unsigned int Elmo_COBID;


/*************************************************函数申明****************************************/
int Elmo_ValueToData(int value, int type, char* data);
int Elmo_DataToValue(char* data, int type, int* value);
void Elmo_WriteCanDataSDO(char ID, int object, int data, char* Elmo_SendBase);
void Elmo_ReadCanDataSDO(char ID, int object, char* Elmo_SendBase);
void Elmo_SYNC(char* Elmo_SendBase);
void Elmo_NMT(char ID, char object,char* Elmo_SendBase);
void Elmo_WriteCanDataPDO1(char ID, int object, int data,char* Elmo_SendBase);
void Elmo_WriteCanDataPDO2(char ID, int object, int data,char* Elmo_SendBase);
int    Elmo_AnalyCanData(int Elmo_RID,char* Elmo_ReceiveBase, int* IDdata);





#endif // __Elmo_gold_canopen_h
