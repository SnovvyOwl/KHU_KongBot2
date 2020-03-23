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
            
       
        //fout<<<<roll<<"\t"<<pitch<<"\t"<<yaw<<endl;
    do{
        AHRSread(roll,pitch,yaw,AHRS);//FUNCTION SENSOR NEED
        switch (int(CMD)){
        //CMD to NANO
        // * IDU MOTOR INPUT , PENDULUM RIGHT MOTOR INPUT , PENDDULUM LEFT MOTOR INPUT, CONTROLL ROLL MOTOR INPUT
        //string CMD="* 1500 1500 1500 1500\n"; //fake CMD
        //serialPuts(NanoCMD,CMD.c_str());
        // keyboard INPUT
        // w (foward_1) 
        // s (backward_1
        // a (chage roll) - direction  
        // d (chage roll) + direction 
        // j (change yaw) +
        // k (chage yaw ) -
        // W (Foward_2)]
        // S (Backward_2)
            case 119 :
                //CMD=w
                cout<< "go\n";
                break;
            case 115 :
                //CMD=s
                cout<<"back\n";
                break;

            case 87:
                //CMD=W
                cout<< "GO\n"<<endl;
                break;
            
            case 83:
                //CMD=S
                cout<< "BACK\n"<<endl;
                break;
            
            case 97:
                //CMD=a
                cout<< "chage roll - direction \n"<<endl;
                break;
            case 100:
                //CMD=d
                cout<< "chage roll + direction \n"<<endl;
                break;
            case 106:
                //CMD=j
                cout<< "chage yaw  +15 degree direction \n"<<endl;
                break;
            case 107:
                //CMD=k
                cout<< "chage yaw  -15 degree direction \n"<<endl;
                break;

        }


    } while (CMD != 'q');
    cout << "quit" << endl;  
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