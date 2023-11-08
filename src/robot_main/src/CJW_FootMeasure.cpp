#include "CJW_FootMeasure.h"

/*******************************parameter*********************************************/
int Foot_fd;
double FootF[8]={0};



/*****************************function***********************************************/
int Foot_Init(const char *portName , int baud)
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
            Foot_fd=fd;
            error=RIGHT;
        }
    }
    else
    {
        printf("error port name!");
    }
    return error;
}

int Foot_Read(int fd, unsigned char* buff,int len)
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

int Foot_Analysis(double* FooF)
{
    int footn=0;
    unsigned char data[FOOT_MAX_BUFF];
    int datan=read(Foot_fd, data, FOOT_MAX_BUFF);
    //printf("the n is %d\n",datan);
    if(datan>=FOOT_PACK_N)
    {
        for(int searchi=0;searchi<=(datan-FOOT_PACK_N);searchi++)
        {
            if((data[searchi]==0xFF)&&(data[searchi+1]==0xFF))
            {
                for(int n=0;n<FOOT_N;n++)
                {
                    int n2=2*n+searchi+2;
                    int temp=(unsigned int)data[n2]*256+data[n2+1];
                    FooF[n]=1.0*temp/FOOT_MAX_UNIT*FOOT_MAX_FORCE;
                    //printf("the data %d is %.3f\n",n,FooF[n]);
                }
                return RIGHT;
                // searchi+=FOOT_PACK_N;
            }
        }
    }
    return ERROR;
}

//int serialWrite(int fd, unsigned char* data, int len)
//{
//  int rv;
//  int length = len;
//  int totalsent = 0;
//  while (totalsent < length)
//  {
//    rv = write(fd, data + totalsent, length);
//    if (rv < 0)
//      printf("write(): error writing - trying again - ");
//    else
//      totalsent += rv;
//  }
//  return rv;
//}

