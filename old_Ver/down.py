import RPi.GPIO as GPIO
import motor
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

PLServo=motor.Servo(14,6.77)
PRServo=motor.Servo(12,6.77)
CamServo=motor.Servo(19,6.77)

PLServo.turn(0)
PRServo.turn(0)
CamServo.turn(0)
PLServo.stop()
PRServo.stop()
CamServo.stop()
GPIO.cleanup()