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
using namespace std;
using namespace cv;
//SENSOR
#define PhaseA 21 //Encoder A
#define PhaseB 22 //Encoder B
#define  BUFF_SIZE 8

float encoder_pulse = 0;
float angle = 0;
int encoder_pos = 0;
bool State_A = 0;
bool State_B = 0;
void Interrupt_A();
void Interrupt_B(); 

class Client{
    private:
        int client=0;
        int AHRS;//Serial
        int Nano; //NANO
        struct sockaddr_in server_addr;
        string msgReceive="";
        string msgSend="";
        string nanoCMD;
        struct hostent *he;
        string AHRSport;
        int AHRSbaud;
        int NanoBaud;
        string NanoPort;
        float tiltAngle=0;
        string RPY="";
        uint32_t past = 0;
        uint32_t now = 0;
        uint32_t controlPeriod = 20; //20ms
        uint32_t time= now-past;
        float angularVel = 0;
        encoder_pulse = (float)360 / (3600 * 4)*1000; //3600 PPR	
        float anglePast = angle;
	    float angleNow = angle;
        int penL;
        int penR;
        int idu;
        int tilt;

    public:
        Client(const char *hostname, const int _port, const char *_AHRSport,int _AHRSbaud, const char *_NanoPort,int _NanoBaud)
        {   
            AHRSport=_AHRSport;
            AHRSbaud=_AHRSbaud;
            NanoPort=_NanoPort;
            NanoBaud=_NanoBaud;
            he=gethostbyname(hostname); // server url
            client = socket( AF_INET, SOCK_STREAM, 0);
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(_port);
            server_addr.sin_addr.s_addr=*(long*)(he->h_addr_list[0]);
            if(client==-1){
                cerr<< "\n Socket creation error \n";
                sock_receive="quit";
				exit(1);
            }
            if (connect(client, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){    
                cerr<<"\nConnection Failed \n"; 
                sock_receive="quit";
				exit(1);
            }
            startClient();
        }
        void startClient(){
            pinMode(PhaseA, INPUT);
	        pinMode(PhaseB, INPUT);
	        wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);
	        wiringPiISR(PhaseB, INT_EDGE_BOTH, &Interrupt_B);
            if((AHRS = serialOpen(AHRSport,AHRSbaud))<0){
                cerr<<"Unable to open AHRS"<<endl;
	            exit(1);
            }
            if((Nano=serialOpen(NanoPort,NanoBaud))<0){
                cerr<<"Unable to open Arduino"<<endl;
	            exit(1);
            }
            initNano();
            runClient();
        }
        void AHRSread(){   
            int rawdata;
            string data;
            do{
                rawdata=serialGetchar(AHRS);
                data+=(char)rawdata;
            }while(rawdata!=10);//init ASCII 42 == "*"
            RPY=data;
        }
        
        void initNano(){
            //CONNECT ARDUINO [INIT..]
            int rawdata;
            string data="";
            do{
                rawdata=serialGetchar(Nano);
                data+=(char)rawdata;
            }while(rawdata!=42);
            cout<<data<<endl; //"Arduino is Ready*"
            //init IDU Stable...
            //####################################
            string nanoCMD="1500";

            serialPuts(Nano,nanoCMD.c_str());//fake CMD
            //#######################################
            data="";
            do{
                rawdata=serialGetchar(Nano);
                data+=(char)rawdata;
            }while(rawdata!=64);
            cout<<data<<endl; //"KHU KongBot2 is Ready@"
        }
        void CAM(string &CMD){
            //CAMERA RECORD [THREAD 3]
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

            } while(CMD!="q");
            Camera.release();
        }
        void runClient(){
            now = past = millis();
	        float anglePast = angle;
	        float angleNow = angle;
            string CMD; //CMD
            thread inputCMD(&input, ref(CMD));//INPUT command Thread.....
            thread camera(&CAM,ref(CMD));
            camera.detach();
            inputCMD.detach();
            //fout<<roll<<"\t"<<pitch<<"\t"<<yaw<<angle<<angleVel<<endl
            thread receive_thread([&](){receiving();});
            thread send_thread([&](){sending();});
            receive_thread.detach();
            send_thread.detach();
            do{
                AHRSread();
                if((time)>controlPeriod){
			        angleNow = angle;
			        angularVel = (angleNow-anglePast)/(time) * 1000;
			        past=millis();
                    time= now-past;
			        anglePast=angle;
		        }
		        now=millis();//FUNCTION SENSOR NEED
                //cout << "Encoder Pos : " << encoder_pos << "\tAngle : " << angle << "\t Vel : " << angularVel << "\n";
                //cout <<roll<<"\t"<<pitch<<"\t"<<yaw<<"\t"<<time/1000<<"\n";
                msgSend=RPY+","+to_string(angle)+","+to_string(tiltAngle)+"\n";
                send(client,msgSend.c_str(),msgSend.size(),0);  
            } while (CMD != 'q');
            //cout << "quit" << endl;  
            serialClose(NanoCMD);
            serialClose(AHRS);
            close(client);
 
        }
};

int main(int argc,char **argv){
    struct sockaddr_in server_addr;
    struct hostent *he;
    string hostname="127.0.0.1";
    int port =13000;
    if(wiringPiSetup()==-1){
        return 1;
    }


    
    //INIT ROBOT
    int penmot=0; //motor pendulum
   
;

}

// THREAD 2, 3
//Socket INPUT [THREAD 2]
void input(string &CMD) {
    char buffer[BUFF_SIZE]={0};
    stringstream sout;
    do {
        read(client,buffer,BUFF_SIZE);
        CMD=buffer;
        sout<<CMD;
        sout>>penR>>penL>>idu>>tilt;
        sout.clear();
    } while (CMD != "q");
    
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
			encoder_pos++;                    // rotate direction : CW
			angle += encoder_pulse;
		}
		else {
			encoder_pos--;                    // rotate direction : CCW
			angle -= encoder_pulse;
		}
	}

	else {
		if (State_A == 0) {
			encoder_pos++;                    // rotate direction : CW
			angle +=  encoder_pulse;
		}
		else {
			encoder_pos--;                    // rotate direction : CCW
			angle -=  encoder_pulse;
		}
	}
}

