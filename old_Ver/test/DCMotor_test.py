import RPi.GPIO as GPIO
import time
  
GPIO.setmode(GPIO.BCM)
  
GPIO_RP = 16
GPIO_RN = 20
GPIO_pwm = 12
  
GPIO.setup(GPIO_RP, GPIO.OUT)
GPIO.setup(GPIO_RN, GPIO.OUT)
GPIO.setup(GPIO_pwm, GPIO.OUT)
pwm = GPIO.PWM(GPIO_pwm, 50)
pwm.start(50)
  
try:
    
    while True:
         print ("forward")
         GPIO.output(GPIO_RP, True)
         GPIO.output(GPIO_RN, False)
         pwm.ChangeDutyCycle(90)
         time.sleep(3) 
          
         print("stop")
         GPIO.output(GPIO_RP, False)
         GPIO.output(GPIO_RN, False)
         pwm.ChangeDutyCycle(90)
         time.sleep(3)
          
         print ("backward")
         GPIO.output(GPIO_RP, False)
         GPIO.output(GPIO_RN, True)
         pwm.ChangeDutyCycle(90)
         time.sleep(3)
          
         print("break")
         pwm.ChangeDutyCycle(90)
         time.sleep(3)
      
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
    sys.exit()