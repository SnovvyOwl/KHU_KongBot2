import motor
import RPi.GPIO as GPIO
import time

'''class lServo(Servo):  
    def mvl(self):
        Servo.pwm=12
        Servo.NDC=6.77
        Servo.turn(3)

class rServo(Servo):
    def mvr(self):
        Servo.pwm=14
        Servo.NDC=6.77
        Servo.turn(-3)
'''
GPIO.setmode(GPIO.BCM)
lServo=motor.Servo(12,6.77)
rServo=motor.Servo(14,6.77)

while True:
    cm = input("Please enter your command:") # receiving user's command by putty
    if(cm=='l'): # now real active coding part
        #lServo.mvl
        lServo.turn(3)
    elif(cm=='r'):
        #rServo.mvr
        rServo.turn(-3)
    else:
        lServo.turn(0)
        rServo.turn(0)