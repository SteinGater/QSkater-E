#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <fcntl.h>
//#include "can4linux.h"
#include "CJW_mycan.h"

#include <iostream>
#include <sstream>
using namespace std;

int Canfd[4]={0};


int canInit(const char* canDevice)
{
    int fd;
    char stdDev[40];
    struct sockaddr_can addr;
    struct ifreq ifr;
    int setflag,getflag,ret =0;

    sprintf(stdDev, "/dev/%s", canDevice);
    printf("opening CAN device %s\n", stdDev);
    if((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
            perror("Error while opening socket");
            return -1;
    }

    strcpy(ifr.ifr_name, canDevice);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
            perror("Error in socket bind");
            return -2;
    }

    getflag =fcntl(fd,F_GETFL,0);
    setflag = getflag|O_NONBLOCK;
    ret = fcntl(fd,F_SETFL,setflag);

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT |CAN_ERR_BUSOFF;
    ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    if (ret!=0)
    {
            printf("setsockopt fail\n");
            return -3;
    }
    else
    {
        return fd;
    }
}

int canWriteOne(int fd, char* data,int id)
{
    struct can_frame frame;
    int state=0;
    int time=0;
    frame.can_id  = id;
    frame.can_dlc = 8;
   frame.data[0] = data[0];
   frame.data[1] = data[1];
   frame.data[2] = data[2];
   frame.data[3] = data[3];
   frame.data[4] = data[4];
   frame.data[5] = data[5];
   frame.data[6] = data[6];
   frame.data[7] = data[7];

   while(state<16)
   {
        state=write(fd, &frame, 16);//sizeof(struct can_frame)
        //printf("%d+%X send\n",state,id);
        usleep(100);
        time++;
        if(time>10)  break;
   }
   return state;
}

int canReadN(int fd,struct can_frame *rx_msg)
{
    int number=0;
    //printf("1receive");
    int state=read(fd, rx_msg,16);

    //printf("1receive");
    while(state>=16)
    {
        if (rx_msg[number].can_id & CAN_ERR_FLAG)
        {
        }
        else
        {
            number++;
        }
        if(number>=RX_MAX) break;
        state=read(fd, &(rx_msg[number]),16);//sizeof (rx_msg)
    }

   return number;
}
