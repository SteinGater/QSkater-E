#ifndef MYCAN_H
#define MYCAN_H

//#include "can4linux.h"

#define TX_MAX 1
#define RX_MAX 100

extern int Canfd[4];

int canInit(const char* canDevice);
int canWriteOne(int fd, char* data,int id);
int canReadN(int fd,struct can_frame *rx_msg);

#endif // CAN_H
