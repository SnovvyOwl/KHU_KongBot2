#include<iostream>
#include<wiringPi.h>

#define PhaseA 2
#define PhaseB 3
using namespace std;

double encoder_pulse = 0;
double angle = 0;
int encoder_pos = 0;

bool State_A = 0;
bool State_B = 0;
void Interrupt_A();
void Interrupt_B();

int main(){
	wiringPiSetup();
	pinMode(PhaseA, INPUT);
	pinMode(PhaseB, INPUT);
	wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);//  encoder pin on interrupt 0 (pin 2)
	wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A); //  encoder pin on interrupt 1 (pin 3)
	while(1) {
		encoder_pulse = (float)360 / (1024 * 4);
		cout << "Encoder Pos : " << encoder_pos << "\tAngle : " << angle << "\tPulse : " << encoder_pulse << endl;
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
