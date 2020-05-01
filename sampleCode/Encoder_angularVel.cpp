#include<iostream>
#include<wiringPi.h>
#define PhaseA 21
#define PhaseB 22
using namespace std;

uint32_t past;
uint32_t now;
uint32_t controlPeriod = 20; //10ms
double encoder_pulse = 0;
double angle = 0;
int encoder_pos = 0;
float vel=0;
bool State_A = 0;
bool State_B = 0;
void Interrupt_A();
void Interrupt_B(); 

int main(){
	wiringPiSetup();
	pinMode(PhaseA, INPUT);
	pinMode(PhaseB, INPUT);
	wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);
	wiringPiISR(PhaseB, INT_EDGE_BOTH, &Interrupt_B);
	encoder_pulse = (float)360 / (3600 * 4); //3600 PPR	
	now = past = millis();
	float anglePast = angle;
	float angleNow = angle;
	while(1){
		if((now-past)>controlPeriod){
			angleNow =angle;
			vel = (angleNow-anglePast)/(now-past);
			now = past=millis();
			anglePast=angle;
		}
		now=millis();
		cout << "Encoder Pos : " << encoder_pos << "\tAngle : " << angle << "\t Vel : " << vel << "\n";
	}
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
