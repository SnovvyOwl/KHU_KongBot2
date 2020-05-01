#include<iostream>
#include<string.h>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<sstream>
#include<thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
//senseor
#define PhaseA 21 //Encoder A
#define PhaseB 22 //Encoder B
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
void CAM(char &CMD);

//Interupt
void Interrupt_A();
void Interrupt_B(); 

//Global Varialbe;
double encoder_pulse = 0;
double angle = 0;
int encoder_pos = 0;
bool State_A = 0;
bool State_B = 0;

int main(int argc,char **argv){
    if(wiringPiSetup()==-1){
        return 1;
    }
    int AHRS;//Serial
    int NanoCMD; //NANO
    char CMD; //CMD
    thread inputCMD(&input, ref(CMD));//INPUT command Thread.....
    thread camera(&CAM,ref(CMD));
    camera.detach();
    inputCMD.detach();
    if((AHRS = serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open AHRS"<<endl;
	      return 1;
    }
      if((NanoCMD=serialOpen("/dev/ttyUSB1",115200))<0){
        cerr<<"Unable to open Arduino"<<endl;
	      return 1;
    }
    //INIT ROBOT
    float roll =0;
    float pitch = 0;
    float yaw = 0;
    uint32_t past = 0;
    uint32_t now = 0;
    uint32_t controlPeriod = 20; //20ms
    double angularVel = 0;
    encoder_pulse = (float)360 / (3600 * 4); //3600 PPR	

    initNano(NanoCMD);
    now = past = millis();
	double anglePast = angle;
	double angleNow = angle;         
       
    //fout<<<<roll<<"\t"<<pitch<<"\t"<<yaw<<angle<<angleVel<<endl;
    do{
        AHRSread(roll,pitch,yaw,AHRS);
        if((now-past)>controlPeriod){
			angleNow =angle;
			vel = (angleNow-anglePast)/(now-past);
			now = past=millis();
			anglePast=angle;
		}
		now=millis();//FUNCTION SENSOR NEED
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
                cout<< "GO\n";
                break;
            
            case 83:
                //CMD=S
                cout<< "BACK\n";
                break;
            
            case 97:
                //CMD=a
                cout<< "chage roll - direction \n";
                break;
            case 100:
                //CMD=d
                cout<< "chage roll + direction \n";
                break;
            case 106:
                //CMD=j
                cout<< "chage yaw  +15 degree direction \n";
                break;
            case 107:
                //CMD=k
                cout<< "chage yaw  -15 degree direction \n";
                break;
            case 113:
                break;
            default:
                cout<< "Wrong OR EMPTY CMD\n";
                break;
        }

    } while (CMD != 'q');
    cout << "quit" << endl;  
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
void CAM(char &CMD){
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 240);
    if (!Camera.open()) {
        cerr<<"Error opening the camera"<<endl; // Connect Camera..
        return;
    }
    cv::VideoWriter outputVideo;
    cv::Size frameSize(320,240);
    int fps = 25;
    outputVideo.open("output.avi", cv::VideoWriter::fourcc('X','V','I','D'),fps, frameSize, true);
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " <<"output.avi" << endl;
        return;
    }
    
    do{
        Camera.grab();
        Camera.retrieve ( image);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::imshow( "test", image );
        outputVideo.write(image);

    } while(CMD!='q');
    Camera.release();
}

oid Interrupt_A() {

	State_A = digitalRead(PhaseA);
	State_B = digitalRead(PhaseB);

	if (State_A == 1) {

		if (State_B == 0) {
			encoder_pos++;                    // 회전방향 : CW
			angle +=  encoder_pulse;
		}
		else {
			encoder_pos--;                    // 회전방향 : CCW
			angle -= encoder_pulse;
		}
	}

	else {
		if (State_B == 1) {
			encoder_pos++;                    // 회전방향 : CW
			angle += encoder_pulse;
		}
		else {
			encoder_pos--;                    // 회전방향 : CCW
			angle -= encoder_pulse;
		}
	}

}

void Interrupt_B() {

	State_A = digitalRead(PhaseA);
	State_B = digitalRead(PhaseB);

	if (State_B == 1) {

		if (State_A == 1) {
			encoder_pos++;                    // 회전방향 : CW
			angle += encoder_pulse;
		}
		else {
			encoder_pos--;                    // 회전방향 : CCW
			angle -= encoder_pulse;
		}
	}

	else {
		if (State_A == 0) {
			encoder_pos++;                    // 회전방향 : CW
			angle +=  encoder_pulse;
		}
		else {
			encoder_pos--;                    // 회전방향 : CCW
			angle -=  encoder_pulse;
		}
	}
}
