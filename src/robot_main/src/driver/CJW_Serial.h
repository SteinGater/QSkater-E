#ifndef CJW_SERIAL
#define CJW_SERIAL


int serialInit(const char *portName/*, int baud*/);
int serialInitB(const char *portName, int baud);
int serialRead(int fd, unsigned char* buff,int len);
int serialWrite(int fd, /*uint8_t*/unsigned char* data, int len);

int iii();


//seriel parament
extern int fd_servo0;
extern int fd_servo1;
extern int fd_ADmodel;
#endif // CJW_SERIAL



