import RPi.GPIO as gpio
import time
leftpwm = 12 #pwm pin number
frequency = 50 
gpio.setmode(gpio.BCM)
gpio.setup(leftpwm,gpio.OUT)
lpwm = gpio.PWM(leftpwm,frequency)
lpwm.start(0)
try:
    
    dc = 0 #ducy cycle
    min_dc = 1.0 # minimum value of duty cycle this need experiment
    max_dc = 10  # maximum value of duty cycle this need experiment
    change_step = 10 #duty cycle change step
    change = float((max_dc-min_dc)/change_step)
    toggle = 1
    delay = 0.4
    dc = min_dc
    
    while True:
    
    lpwm.ChangeDutyCycle(dc)
    
    time.sleep(delay)
    
    if dc +change <=max_dc and toggle:
    dc += change
    
    elif dc -change >=min_dc:
    dc -=change
    toggle = 0
    
    else:
    dc += change
    toggle =1
    
except KeyboardInterrupt:
    lpwm.stop()
    gpio.cleanup()