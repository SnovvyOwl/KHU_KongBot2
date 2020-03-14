
#include<stdio.h>
#include<string.h>
#include<errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
using namespace std;
int main()
{
    int fd;
    int data;
    if((fd=serialOpen("/dev/ttyS0",115200))<0){
        fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
	return 1;
    }
    printf("\nRasberry Pi UART Test");
    for(int i=0;i<=50;i++){
        data=serialGetchar(fd);
        printf("%d",data);
    }
    return 0;
} 
