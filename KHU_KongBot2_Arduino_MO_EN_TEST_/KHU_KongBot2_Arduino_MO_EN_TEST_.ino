///
#include <Servo.h>
//Define Pin NUMBER
#define PEND_R 10
#define PEND_L 11
#define IDU 9
#define CTRL_ROLL 6
//DEFINE FUNTION
void init_motor();
void reSYNC_motor();
void active_motor();
//DEFINE CMD 
int CMD_IDU;
int CMD_PEND_R;
int CMD_PEND_L;
int CMD_CTRL_ROLL;
int reSYNC=0;
int ori_angular = 1500;
int angular;

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

void setup() {
  Serial.begin(115200);//Baudrate는 추후 결정!
  Serial.println("Arduino is Ready*");
  init_motor();
  
}

void loop() {
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
   
    /*if(reSYNC){
      reSYNC_motor();
    }*/
       
  }

}
void init_motor(){
  pendR.attach(PEND_R);
  pendL.attach(PEND_L);
  idu.attach(IDU);
  ctrl_roll.attach(CTRL_ROLL);
  pendR.writeMicroseconds(1500);//600~2400
  pendL.writeMicroseconds(1500);//600~2400
  ctrl_roll.writeMicroseconds(1500);//600~2400
  do{
    CMD_IDU=Serial.parseInt();
    idu.writeMicroseconds(CMD_IDU);
    }while(CMD_IDU!=1500);  
  Serial.write("KHU KONG BOT2 is READY@");
  delay(1000);
  active_motor();
 }

 /*void reSYNC_motor(){
  CMD_IDU=idu.read();
  CMD_PEND_R=pendR.read();
  CMD_PEND_L=pendL.read();
  CMD_CTRL_ROLL=ctrl_roll.read();
  CMD=String(CMD_IDU)+String(',')+String(CMD_PEND_R)+String(',')+String(CMD_PEND_L)+String(',') +String(CMD_CTRL_ROLL);  
  Serial.write(CMD.c_str());
  }*/

 void active_motor(){
  Serial.write("enter pendulum target angle");
  //angular = Serial.read();//
  //ori_angular = angular * 10;//
  for(int i = 0; i<2; i++){
    ori_angular = 86;
    pendR.writeMicroseconds(1500+ori_angular);
    pendL.writeMicroseconds(1500+ori_angular);
    delay(800);
    ori_angular = 0;
     pendR.writeMicroseconds(1500+ori_angular);
     delay(800);
    Serial.println(i);
    }

   for(int j = 0; j<3; j++){
    ori_angular = 170;
    pendR.writeMicroseconds(1500+ori_angular);
    pendL.writeMicroseconds(1500+ori_angular);
    delay(800);
    ori_angular = 0;
    pendR.writeMicroseconds(1500+ori_angular);
    delay(800);
    Serial.println(j);
   }
  } 
