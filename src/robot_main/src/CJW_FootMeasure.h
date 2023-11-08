#ifndef CJW_FOOTMEASURE
#define CJW_FOOTMEASURE

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
#include "UserRobotStructure.h"
/*******************************parameter*********************************************/
#define FOOT_MAX_BUFF   1000
#define FOOT_PACK_N     18
#define FOOT_N          8
#define FOOT_MAX_UNIT   4096
#define FOOT_MAX_FORCE  100


/*****************************function***********************************************/
int Foot_Init(const char *portName, int baud);
int Foot_Read(int fd, unsigned char* buff,int len);
int Foot_Analysis(double* FooF);
//int serialWrite(int fd, /*uint8_t*/unsigned char* data, int len);

#endif



