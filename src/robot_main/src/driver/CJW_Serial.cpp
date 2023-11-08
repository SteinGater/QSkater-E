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
#include "CJW_Serial.h"

// Global viriable.
int fd_servo0=0;
int fd_servo1=0;
int fd_ADmodel=0;

int serialInit(const char *portName /*, int baud*/)
{
  struct termios options;
  int fd;

  if (*portName == '/')       // linux serial port names always begin with /dev
  {
    printf("Opening serial port %s\n", portName);

    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
      // Could not open the port.
      perror("init(): Unable to open serial port - ");
    }
    else
    {
      // Sets the read() function to return NOW and not wait for data to enter
      // buffer if there isn't anything there.
      fcntl(fd, F_SETFL, FNDELAY);

      // Configure port for 8N1 transmission, 115200 baud, SW flow control.
      tcgetattr(fd, &options);
      cfsetispeed(&options, B1000000);
      cfsetospeed(&options, B1000000);
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
    }
    return fd;
  }
  else
  {
      printf("error port name!");
      return 0;
  }
}

int serialInitB(const char *portName, int baud)
{
  struct termios options;
  int fd;

  if (*portName == '/')       // linux serial port names always begin with /dev
  {
    printf("Opening serial port %s\n", portName);

    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
      // Could not open the port.
      perror("init(): Unable to open serial port - ");
    }
    else
    {
      // Sets the read() function to return NOW and not wait for data to enter
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
    }
    return fd;
  }
  else
  {
      printf("error port name!");
      return 0;
  }
}

int serialRead(int fd, unsigned char* buff,int len)
{
  unsigned char c;
  unsigned int i;
  int rv;
//  rv = read(fd, &c, 1); // read one byte
  rv = read(fd, buff, len);

  //i = c;          // convert byte to an int for return
  //if (rv > 0)
  //  return rv;     // return the character read
  if (rv < 0)
  {
    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
      perror("elCommRead() error:");
  }

  // return -1 or 0 either if we read nothing, or if read returned negative
  return rv;
}

int serialWrite(int fd, unsigned char* data, int len)
{
  int rv;
  int length = len;
  int totalsent = 0;
  while (totalsent < length)
  {
    rv = write(fd, data + totalsent, length);
    if (rv < 0)
      printf("write(): error writing - trying again - ");
    else
      totalsent += rv;
  }
  return rv;
}

