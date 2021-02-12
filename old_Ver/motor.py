import RPi.GPIO as GPIO
import time

class Servo:
    def __init__(self,pinNum,dutycycle):
        GPIO.setup(pinNum,GPIO.OUT)
        self.pwm=GPIO.PWM(pinNum,50)
        self.NDC=dutycycle
        self.pwm.start(0)

    def turn(self,INPUT):
        real=self.NDC+INPUT
        self.pwm.ChangeDutyCycle(real)
        time.sleep(0.01)
    
    def stop(self):
        self.turn(0)
        self.pwm.stop()

class DCmotor:
    def __init__(self,IN1,IN2):
        self.GPIO_IN1=IN1
        self.GPIO_IN2=IN2
        GPIO.setup(self.GPIO_IN1,GPIO.OUT)
        GPIO.setup(self.GPIO_IN2,GPIO.OUT)
        self.pwm0=GPIO.PWM(self.GPIO_IN1,50)
        self.pwm1=GPIO.PWM(self.GPIO_IN2,50)
        self.pwm0.start(50)
        self.pwm1.start(50)
        
    def CW(self,INPUT):
        self.pwm0.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(INPUT)
        time.sleep(0.01)

    def CCW(self,INPUT):
        self.pwm0.ChangeDutyCycle(INPUT)
        self.pwm1.ChangeDutyCycle(0)
        time.sleep(0.01)
        
    def stop(self):
        self.pwm0.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm0.stop()
        self.pwm1.stop()