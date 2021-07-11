#include<iostream>
#include<string.h>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<sstream>
#include<thread>
#include<raspicam/raspicam_cv.h>
#include<opencv2/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<state.h>
#include<random>
using namespace std;
using namespace cv;
/*
ms=0.51
mi=1.00
mp=0.36*2
Rs=0.15
Js=3.89*10**-3
Ji=4.42*10**-3
Jp=3.4*10**-4*2
g=9.81
lp=0.1
*/
//SENSOR
#define PhaseA 21 //Encoder A
#define PhaseB 22 //Encoder B
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
#define SIGMA_V 10 //Measurement Noise
#define T 0.1
void AHRSread(float &ROLL,float &PITCH,float &YAW,const int &fd);

void KF_filter(Mat &x_bar, Mat &x_hat, Mat &X, Mat &U, Mat &z_hat, Mat &Z,Mat &p_bar,Mat &p_hat,Mat &S,Matx44d Q, double R, Matx44d &Qf,Mat &K);//KF
//GLOBAL VARIABLE
//NORMAL DISTRIBUTION RANDOM NUMBER GENERATOR
default_random_engine generator;
normal_distribution<double> dist_W(0,1.0);
normal_distribution<double> dist_V(0,1.0);
double W=0;//SYSTEM NOISE
double V=0;//SENSOR NOISE
Matx44d F(1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
Matx41d G(-0.0244, -0.4888,7.3529 ,147.0588);
Matx14d H(0,1,0,0);
//penL=Pendulum(0.0);
//penR=Pendulum(0.0);
pen=Pendulum(0.0);

//Robot
void initNano(const int &fd);
float change_Vel(float desire_speed,float real_speed,float time);
void change2Yaw(float desire_yaw, float real_yaw);
void change2Roll(float desire_roll, float real_roll);
void run (float desire_roll, float real_roll, float desire_speed, float real_speed,float time);

//Thread
void input(char &CMD);
void CAM(char &CMD);

//Interupt
void Interrupt_A();
void Interrupt_B(); 

//Global Varialbe;
float encoder_pulse = 0;
float angle = 0;
int encoder_pos = 0;
bool State_A = 0;
bool State_B = 0;
float err_shellspeed=0;
float pre_err_shellspeed=0;

int main(int argc,char **argv){
    if(wiringPiSetup()==-1){
        return 1;
    }
    int desiredspeed=0;
    int AHRS;//Serial
    int NanoCMD; //NANO
    char CMD; //CMD
    thread inputCMD(&input, ref(CMD));//INPUT command Thread.....
    thread camera(&CAM,ref(CMD));
    camera.detach();
    inputCMD.detach();
    pinMode(PhaseA, INPUT);
	pinMode(PhaseB, INPUT);
	wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);
	wiringPiISR(PhaseB, INT_EDGE_BOTH, &Interrupt_B);
    if((AHRS = serialOpen("/dev/ttyUSB0",115200))<0){
        cerr<<"Unable to open AHRS"<<endl;
	      return 1;
    }
      if((NanoCMD=serialOpen("/dev/ttyACM0",115200))<0){
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
    float angularVel = 0;
    encoder_pulse = (float)360 / (3600 * 4)*1000; //3600 PPR	

    initNano(NanoCMD);
    now = past = millis();
	float anglePast = angle;
	float angleNow = angle;         
    // INIT KALMAN FILTER
    //Initial Conditions
    Mat X=(Mat_<double>(4,1)<<0,0,0,0);
    Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
    Mat P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    sqrt(P,P);
    double W=dist_W(generator);
    Mat X_bar;
    X_bar.copySize(P.mul(W).diag());
    X_bar=X+P.mul(W).diag();
    P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    Mat p_bar = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    Mat p_hat = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    //Noise
    Matx44d Q(pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); //SYSYTEM NOISE 바꿀 필요가 있어보임
    double R= SIGMA_V*SIGMA_V;
    Mat u=(Mat_<double>(1,1)<<0);
    Matx44d Qf=Q;
    Mat z_hat;
    Mat Z;
    Mat K;
    Mat S;  
    //fout<<roll<<"\t"<<pitch<<"\t"<<yaw<<angle<<angleVel<<endl;
    do{
        AHRSread(roll,pitch,yaw,AHRS);
        if((now-past)>controlPeriod){
			angleNow = angle;
			angularVel = (angleNow-anglePast)/(now-past) * 1000;
			past=millis();
			anglePast=angle;
		}
		now=millis();//FUNCTION SENSOR NEED
        cout << "Encoder Pos : " << encoder_pos << "\tAngle : " << angle << "\t Vel : " << angularVel << "\n";
        cout <<roll<<"\t"<<pitch<<"\t"<<yaw<<"\n";
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
                desiredspeed+=10;
                change_Vel(desiredspeed,angularVel,1.0);
                
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

// THREAD 2, 3
//KEBOARD INPUT [THREAD 2]
void input(char &CMD) {
    do {
        cin >> CMD;
    } while (CMD != 'q');
    
}

//CAMERA RECORD [THREAD 3]
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

//SENSOR 
//DETECT ROLL,PITCH,YAW, Gx,Gy,Gz [AHRS]
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

//INTERRUPT pin A [ENCODER]
void Interrupt_A() {

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
//INTERRUPT pin B [ENCODER]
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

//ROBOT MOVE
//CONNECT ARDUINO [INIT..]
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


//SPEED CONTROL
float change_Vel(float &desire_speed,float real_speed,float time){
    /*
        PID CONTROLLER
                     Tz      z-1
        PID(z)=Kp+Ki---- +Kd-----
                     z-1     Tz
    */
    /*

               0.1308 z + 0.1308           0.1308 z^-1 + 0.1308z^-2
        G(Z)= -----------------     =     ------------------------- 
              z^2 + 1.993 z + 1           1 + 1.993 z^-1 + Z^-2
    */
    /*
        0.1308*(z + 1)*(Kd*(z - 1)**2 + Ki*T**2*z**2 + Kp*T*z*(z - 1))/(T*z*(z - 1)*(z**2 + 1.993*z + 1))
    */
    float kp=20;
    float ki=10;
    float kd=5;
    float gain=0;
    float G=0;
    pre_err_shellspeed=err_shellspeed;
    err_shellspeed=desire_speed-real_speed;
    gain=kp*err_shellspeed+ki*(err_shellspeed*time)+kd*(err_shellspeed-pre_err_shellspeed);
    G=pen.shell2pen(gain);
    return G;
}

//CHANGE YAW
void change_Yaw(float desire_yaw, float real_yaw){
    cout<<desire_yaw<<endl;
}

// CHANGE ROLL
void change_Roll(float desire_roll, float real_roll){
    float error=0;
    float kp=0;
    float ki=0;
    float kd=0;
    float G=0;
    error=desire_roll-real_roll;
    G= kp*error +ki*(error*time)+kd*err-preerror
    cout<<desire_roll<<endl;
}

//CURVE 
void run (float desire_roll, float real_roll, float desire_speed, float real_speed,float time){

    //change_Roll(desire_roll,real_roll);
    //change_Vel(desire_speed,real_speed,time);
}

//KALMAN FILTER
void KF_filter(Mat &x_bar, Mat &x_hat, Mat &X, Mat &U, Mat &z_hat, Mat &Z,Mat &p_bar,Mat &p_hat,Mat &S,Matx44d Q, double R, Matx44d &Qf,Mat &K){
    //Shell 속도 추정 -> class pendulum
    double V= dist_V(generator);
    Z=H*X+sqrt(R)*V; //MESUREMENT MODEL
    //==================
    //MESUREMENT UPDATE
    //=================
    z_hat=H*x_bar;
    S=H*p_bar*H.t()+R;
    p_hat=p_bar-p_bar*H.t()*S.inv()*H*p_bar;
    K=p_bar*H.t()*S.inv();
    x_hat=x_bar+K*(Z-z_hat);
    //==================
    // TIME UPDATE
    // =================
    x_bar=F*x_hat+G*U;
    p_bar=F*p_hat*F.t()+Qf;
    //==================
    //System Dynamics
    //==================
    sqrt(Q,Q);
    double W=dist_W(generator);
    X=F*X+G*U+Q.diag()*W;
}