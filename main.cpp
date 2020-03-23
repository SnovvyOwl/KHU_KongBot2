#include<iostream>
#include<wiringPi.h>
#include<string.h>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<sstream>
#include<thread>
using namespace std;
//senseor
void AHRSread(float &ROLL,float &PITCH,float &YAW,const int &fd);
//Robot
void initNano(const int &fd);
void Foward_1(); 
void Foward_2();
void Backward_1(); 
void Backward_2();
void changeYaw(int yaw);
void changeRoll(int roll);
void run (int roll, int step);
//Thread
void input(char &CMD);
int main(){
    int AHRS;//Serial
    int NanoCMD;
    float roll;
    float pitch;
    float yaw;
    char CMD;
    thread inputCMD(&input, ref(CMD));//INPUT command Thread.....
    inputCMD.detach();
    if((AHRS=serialOpen("/dev/ttyUSB1",115200))<0){
        cerr<<"Unable to open AHRS"<<endl;
	      return 1;
    }
      if((NanoCMD=serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open Arduino"<<endl;
	      return 1;
    }
    
    initNano(NanoCMD);
    
    for(int j=0;j<50;j++){
        AHRSread(roll,pitch,yaw,AHRS);//FUNCTION SENSOR NEED
        cout<<"Roll: "<<roll<<"\t"<<"Pitch: "<<pitch<<"\t"<<"YAW: "<<yaw<<"\n";
        //fout<<<<roll<<"\t"<<pitch<<"\t"<<yaw<<endl;
        
        //CMD to NANO
    // * IDU MOTOR INPUT , PENDULUM RIGHT MOTOR INPUT , PENDDULUM LEFT MOTOR INPUT, CONTROLL ROLL MOTOR INPUT
        //string CMD="* 1500 1500 1500 1500\n"; //fake CMD
        //serialPuts(NanoCMD,CMD.c_str());
        
    }
    serialClose(NanoCMD);
    serialClose(AHRS);
    return 0;
}
void initNano(const int &fd){
    int rawdata;
    string data="";
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=42);
    cout<<data<<endl; //"Arduino is Ready*"
    //init IDU Stable...
    //####################################
    string CMD="1500";

    serialPuts(fd,CMD.c_str());//fake CMD
    //#######################################
    data="";
    do{
        rawdata=serialGetchar(fd);
        data+=(char)rawdata;
    }while(rawdata!=64);
    cout<<data<<endl; //"KHU KongBot2 is Ready@"
}
void AHRSread(float &ROLL,float &PITCH,float &YAW,const int &fd){
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
void input(char &CMD) {
    do {
        cin >> CMD;
    } while (CMD != 'q');
    
}