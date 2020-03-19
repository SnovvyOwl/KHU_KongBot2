#include<iostream>
#include<wiringPi.h>
#include<string.h>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<sstream>
#include<pthread>
using namespace std;
void AHRSread(float &ROLL,float &PITCH,float &YAW,int &fd);
void Foward_1(); 
void Foward_2();
void Backward_1(); 
void Backward_2();
void changeYaw(int yaw);
void changeRoll(int roll);
void run (int roll, int step);

int main(){
    int fd;//Serial
    float roll;
    float pitch;
    float yaw;
    if((fd=serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open serial device"<<endl;
	      return 1;
    }
    for(int j=0;j<50;j++){
        AHRSread(roll,pitch,yaw,fd);//FUNCTION SENSOR NEED
        cout<<"Roll: "<<roll<<"\t"<<"Pitch: "<<pitch<<"\t"<<"YAW: "<<yaw<<endl;
        //fout<<<<roll<<"\t"<<pitch<<"\t"<<yaw<<endl;
    }
    return 0;
}


void AHRSread(float &ROLL,float &PITCH,float &YAW,int &fd){
    int rawdata;
    string data;
    stringstream sout;
    do{
        rawdata=serialGetchar(fd);
    }while(rawdata!=42);//init ASCII 42 == "*"
    
    //DATA ROLL
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=44);
    sout<<data;
    sout>>ROLL;
    sout.str("");data="";
    //DATA PITCH
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=44);
    sout<<data;
    sout>>PITCH;
    sout.str("");data="";
    
    //DATA YAW
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=10);
    sout<<data;
    sout>>YAW;
    sout.str("");data="";
}