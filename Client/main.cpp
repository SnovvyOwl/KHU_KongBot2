#include<wiringPi.h>
#include<iostream>
#include<string.h>
#include<client.h>
using namespace cv;
int main(int argc,char **argv){
    if(wiringPiSetup()==-1){
        return 1;
    }
    Client client("127.0.0.1",13000,"/dev/ttyUSB0",115200,"/dev/ttyACM0",115200);
    client.startClient();
    return 0;
}