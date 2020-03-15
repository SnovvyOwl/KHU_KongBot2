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
    int fd;//Serial
    float roll;
    float pitch;
    float yaw;
    if((fd=serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open serial device"<<endl;
	return 1;
    }
    cout<<"\nRasberry Pi AHRS Test"<<endl;
    for(int j=0;j<10;j++){
        AHRSread(roll,pitch,yaw,fd);//FUNCTION SENSOR NEED
        //cout<<"Roll: "<<roll<<"\t"<<"Pitch: "<<pitch<<"\t"<<"YAW: "<<yaw<<endl;
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
    //cout<<data<<endl;
    sout<<data;
    sout>>ROLL;
    sout.clear();
    data="";
    //DATA PITCH
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=44);
    //cout<<data<<endl;
    sout<<data;
    sout>>PITCH;
    cout<<PITCH<<endl<<endl;
    
    data="";
    
    //DATA YAW
    rawdata=serialGetchar(fd);
    data+=(char)rawdata;
    while(rawdata!=10){
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }
    sout<<data;
    sout>>PITCH;
    data="";
}

