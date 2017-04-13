/* www.chrisheydrick.com
     
   June 23 2012
   CanonicalArduinoRead write a byte to an Arduino, and then
   receives a serially transmitted string in response.
   The call/response Arduino sketch is here:
   https://gist.github.com/2980344
   Arduino sketch details at www.chrisheydrick.com
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string>
#include <ros/ros.h>
#include<geometry_msgs/Quaternion.h>
#define DEBUG 1
  
struct Euler
{
  float roll, pitch, yaw;
};

Euler e;

void ParseEuler(char * buf, int bufLen)
{
  int index = 0;
  int count = 0;
  char number[10];
  
  for(int i = 0; i < bufLen; i++)
  {
    if(buf[i] != ',' && i != bufLen - 1)
    {
      number[index] = buf[i];
      index++;
    }
    else
    {
      number[index + 1] = '\0';
      switch(count)
      {
      case 0:
	e.roll = atof(number); 
	printf("Roll:%f\n",e.roll);
	break;
      case 1:
	e.pitch = atof(number); 
	printf("Pitch:%f\n",e.pitch);
	break;
      case 2:
	e.yaw = atof(number); 
	printf("Yaw:%f\n",e.yaw);
	break;
      }

      count++;
      i++;
      index = 0;
    }
  }

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Euler_ros");
    ros::NodeHandle nh;
    
    
    int fd, n, i;
    const int bufSize = 128;

    char tarCmd [] = {':', '0', '0', '\n'};
    char QuatCmd[] = {':', '0','0' ,'\n'};
    char EulerCmd[] = {':', '1','0' ,'\n'};
    char buf[bufSize];

    struct termios toptions;

    /* open serial port */
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n", fd);

    /* wait for the Arduino to reboot */
    usleep(3500000);

    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 9600 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    //Tare
    write(fd, &tarCmd[0], 1);
    write(fd, &tarCmd[1], 1);
    //write(fd, &tarCmd[2], 1);  
    write(fd, &tarCmd[3], 1);

    printf("%s\n", tarCmd);

    memset(buf, '\0', bufSize);
    n = read(fd, buf, bufSize);

    printf("Response: %s", buf);
    
    geometry_msgs::Quaternion eulerAngles;
    ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("Robot/RPY",1000);
    
    usleep(10000000);
    while(true)
    {
        //Get Quaternion
        write(fd, &EulerCmd[0], 1);
        write(fd, &EulerCmd[1], 1);
        //write(fd, &EulerCmd[2], 1);  
        write(fd, &EulerCmd[3], 1);

        //Clear buffer and read incomming bytes
        memset(buf, '\0', bufSize);
        n = read(fd, buf, bufSize);
        system("clear");
        /* insert terminating zero in the string */
        ParseEuler(buf, n);
        buf[n] = 0;
        eulerAngles.w = e.pitch;
        eulerAngles.x = e.roll;
        eulerAngles.y = e.yaw;
        pub.publish(eulerAngles);//publish Euler angles in the form of geometry_msgs quaternion to make it easy.
        usleep(100000);
    }
    return 0;
}
