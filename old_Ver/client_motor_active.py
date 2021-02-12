import socket
import RPi.GPIO as GPIO
import motor


client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('192.168.137.194',8080))

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
lServo=motor.Servo(12,6.77)
rServo=motor.Servo(14,6.77)

while True:
    client.send('I am CLIENT\n'.encode())
    from_server = client.recv(4096)
    print (from_server)

    if from_server == b'w':
        print ('Forward')
        lServo.turn(3)
        rServo.turn(-3)
        
    elif from_server == b's':
        print ('Backward')
        lServo.turn(-3)
        rServo.turn(3)
        
    elif from_server == b'a':
        print ('Turn left')
        #DC turn left
    
    elif from_server == b'd':
        print ('Turn right')
        #DC turn right
        
    elif from_server == b' ':
        print ('Stop')
        
        
    else :
        print ('wrong command')
        lServo.turn(0)

GPIO.cleanup()
