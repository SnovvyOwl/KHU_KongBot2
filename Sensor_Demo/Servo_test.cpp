#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <softPwm.h>
 
int main ()
{
  int pos = 10 ;
  int dir = 1 ;
  if (wiringPiSetup() == -1) exit(1) ;  //init wiringPi
 
  pinMode(0, OUTPUT) ; 
  digitalWrite(0, LOW) ;  // 0 pin output LOW voltage
  softPwmCreate(0, 0, 200) ; //SoftPWM... 
  pinMode(1,OUTPUT);
  while(1) {
    pos += dir ;
    if (pos < 10 || pos > 20) dir *= -1 ;
    softPwmWrite(0, pos) ;
    pwmWrite(1,pos);
    delay(190) ;
  }
  return 0 ;
}