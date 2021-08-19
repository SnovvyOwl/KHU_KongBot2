#include<wiringPi.h>
#include<iostream>
#include<string.h>
#include<client.h>
using namespace cv;
int main(int argc,char **argv){
    if(wiringPiSetup()==-1){
        return 1;
    }
    Client client("192.168.35.98",13000);
    //client.startClient("/dev/ttyUSB0",115200,"/dev/ttyACM0",115200);
    return 0;
}
