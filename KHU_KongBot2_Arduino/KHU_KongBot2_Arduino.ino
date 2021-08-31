#include <Servo.h>
//Define Pin NUMBER
#define PEND_R 10
#define PEND_L 11
#define IDU 9
#define CTRL_ROLL 6
//DEFINE FUNTION
void init_motor();
void reSYNC_motor();
//DEFINE CMD 
int CMD_IDU;
int CMD_PEND_R;
int CMD_PEND_L;
int CMD_CTRL_ROLL;
int reSYNC=0;
int tiltTheta=0;
//DEFINE SERVO
Servo pendR;
Servo pendL;
Servo idu;
Servo ctrl_roll;
/* Command : IDU PEND_R PEND_L CTRL_ROLL
 1. init state 
    RPI command Only IDU...
 2. Running state
    RPI Calculate ALL parameter..
    send servo Parameter to Arduino
 3. reSync
*/
String CMD;
String tiltAngle;
void setup() {
  	Serial.begin(115200);//Baudrate는 추후 결정!
  	Serial.write("Arduino is Ready*");
  	init_motor();
}

void loop() {
  	tiltTheta=analogRead(A0);
	tiltAngle=String(tiltTheta)+"*";
  	if(Serial.available()){
    	CMD_IDU=Serial.parseInt();
    	CMD_PEND_R=Serial.parseInt();
    	CMD_PEND_L=Serial.parseInt();
    	CMD_CTRL_ROLL=Serial.parseInt();
    	reSYNC=Serial.parseInt();
    	idu.writeMicroseconds(CMD_IDU);
    	pendR.writeMicroseconds(CMD_PEND_R);//0~180
    	pendL.writeMicroseconds(CMD_PEND_L);//0~180
    	ctrl_roll.writeMicroseconds(CMD_CTRL_ROLL);//0~180
    	Serial.write(tiltAngle.c_str());
		if(reSYNC){
      	reSYNC_motor();
    	}
  	}
}

void init_motor(){
  	pendR.attach(PEND_R);
  	pendL.attach(PEND_L);
  	idu.attach(IDU);
  	ctrl_roll.attach(CTRL_ROLL);
  	pendR.writeMicroseconds(560);//700~2300
  	pendL.writeMicroseconds(1500);//700~2300
  	ctrl_roll.writeMicroseconds(1500);//700~2300
  	do{
    	CMD_IDU=Serial.parseInt();
    	idu.writeMicroseconds(CMD_IDU);
    }while(CMD_IDU!=1500);  
  	Serial.write("KHU KONG BOT2 is READY@");
}

void reSYNC_motor(){
  	CMD_IDU=idu.read();
  	CMD_PEND_R=pendR.read();
  	CMD_PEND_L=pendL.read();
  	CMD_CTRL_ROLL=ctrl_roll.read();
  	CMD=String(CMD_IDU)+String(',')+String(CMD_PEND_R)+String(',')+String(CMD_PEND_L)+String(',') +String(CMD_CTRL_ROLL);  
  	Serial.write(CMD.c_str());
}
