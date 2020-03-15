#include<iostream>
#include<string>
#include<sstream>
//#inlcude<fstream>
#include <wiringPi.h>
#include <wiringSerial.h>
using namespace std;
void AHRSread(float &ROLL,float &PITCH,float &YAW,int &fd);
int main(){
    //ofstream fout;
    //fout.open("AHRS_data.txt");
    
    int fd;

    float roll;
    float pitch;
    float yaw;
    if((fd=serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open serial device"<<endl;
	return 1;
    }
    cout<<"\nRasberry Pi AHRS Test"<<endl;
    for(int j=0;j<30;j++){
        AHRSread(roll,pitch,yaw,fd);
        cout<<"Roll: "<<roll<<endl<<"Pitch: "<<pitch<<endl<<"YAW: "<<yaw<<endl;
    }
    return 0;
} 
void AHRSread(float &ROLL,float &PITCH,float &YAW,int &fd){
    int i=0;
    int rawdata;
    string data;
    stringstream sout;
    do{
        rawdata=serialGetchar(fd);
    }while(rawdata!=42);//init ASCII 42 == "*"
    while(rawdata!=44){
        rawdata=serialGetchar(fd);
        data=(char)rawdata;
    }//DATA ROLL
    sout<<data;
    sout>>ROLL;
    i=0;data="";
    rawdata=serialGetchar(fd);
    data[i]=(char)rawdata;
    i++;
    while(rawdata!=44){
        rawdata=serialGetchar(fd);
        data[i]=(char)rawdata;
    };//DATA PITCH
    sout<<data;
    sout>>PITCH;
    i=0;data="";
    do{
        rawdata=serialGetchar(fd);
        data[i]=(char)rawdata;
    }while(rawdata!=10);//DATA YAW
    sout<<data;
    sout>>PITCH;
    i=0;data="";
}

