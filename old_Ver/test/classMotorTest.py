import motor
import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#servo1=motor.Servo(12,6.77)
wheelM=motor.DCmotor(12,19)
while True:
    print("positive")
    #servo1.turn(5)
    wheelM.CW(60)
    time.sleep(3)
    print("Negative")
    #servo1.turn(-5)
    wheelM.CCW(60)
    time.sleep(3)
GPIO.cleanup()