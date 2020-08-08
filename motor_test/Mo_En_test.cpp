#include<iostream>
#include<fstream>
#include<thread>
#include<wiringSerial.h>
#include<wiringPi.h>
#include<string.h>
#define PhaseA 21
#define PhaseB 22
using namespace std;

uint32_t past;
uint32_t now;
uint32_t controlPeriod = 20; //20ms
double encoder_pulse = 0;
double angle = 0;
int encoder_pos = 0;

bool State_A = 0;
bool State_B = 0;
float vel=0;
void Interrupt_A();
void Interrupt_B();
void initNano(const int &fd);
void input(char &CMD);
int main(){
	int NanoCMD;
	char CMD='w';
	thread inputCMD(&input,ref(CMD));
	inputCMD.detach();
	ofstream fout;
	fout.open("encoder_data.txt");
	wiringPiSetup();
	pinMode(PhaseA, INPUT);
	pinMode(PhaseB, INPUT);
	wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);
	wiringPiISR(PhaseB, INT_EDGE_BOTH, &Interrupt_B);
	encoder_pulse = (float)360 / (3600 * 4); //3600 ppr
	if((NanoCMD=serialOpen("/dev/ttyACM0",115200))<0){
		cerr<<"Unable to open Arduino"<<endl;
		return 1;
	}
	initNano(NanoCMD);
	//delay(500);
	now = past = millis();
	float anglePast = angle;
	float angleNow = angle;
	do{
		if((now-past)>controlPeriod){
			angleNow =angle;
			vel = (angleNow-anglePast)/(now-past) * 1000;
			now = past=millis();
			anglePast=angle;
		}
		now=millis();
		cout << "Encoder Pos : " << encoder_pos << "\tAngle : " << angle << "\tVel : " << vel << "\n";
		fout<<"time: @"<<now<<"\tEncoder Pos : *"<<encoder_pos<<"\tAngle : $" << angle<< "\tVel :#" <<vel<<"\n";
	}while(CMD!='q');
	cout<<"quit"<<endl;
	string quit="*1500 1500 1500 1500 1500\n";
	serialPuts(NanoCMD, quit.c_str());
	serialClose(NanoCMD);
	fout.close();
	return 0;
}
void initNano(const int &fd){
	int rawdata;
	string data="";
	do{
		rawdata=serialGetchar(fd);
		data+=(char)rawdata;
	}while(rawdata!=42);
	cout<<data<<endl;
	string CMD="1500";
	serialPuts(fd,CMD.c_str());
	data="";
	do{
		rawdata=serialGetchar(fd);
		data+=(char)rawdata;
	}while(rawdata!=64);
	cout<<data<<endl;
}

void input(char &CMD){
	do{
		cin>>CMD;
	}while(CMD!='q');
}

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
