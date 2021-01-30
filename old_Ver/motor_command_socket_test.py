class SocketInfo():
    HOST=''
    PORT=8080
    BUFSIZE
    ADDR(HOST, PORT)

#server code
from socket import *
from data_file import *

ssock=socket(AF_INET, SOCK_STREAM)
ssock.bind(SocketInfo.ADDR)
ssock.listen(5)
csock.addr_info = ssock.accept()

while True:
    try:
        commend=raw_input(">>")
        csock.send(commend)
        print "waiting for response"
        response = csock.recv(SocketInfo.BUFSIZE)
        print response
        
    if response is None : 
        print "waiting for connection"
        csock, addr_info =  ssock.accept()
        print "got connection from", addr_info
    else :
        print "waiting for response"
        print response
    
#client code

from socket import *
from sys import exit
import RPi.GPIO as GPIO
from data_file import SocketInfo
import motor

class SocketInfo(SocketInfo)
    HOST='192.168.137.183' #need to change

csock=socket(AF_INET,SOCK_STREAM)
csock.connect(SocketInfo.ADDR)

GPIO.setmode(GPIO.BCM)
GPIO.setup(Llegpwm, GPIO.OUT)
GPIO.setup(Rlegpwm, GPIO.OUT)
lServo=motor.Servo(12,6.77)
rServo=motor.Servo(14,6.77)

while true:
    
    commend=csock.recv(SocketInfo.BUFSIZE)
    response = 'connected'
    
    if commend=='w':
            print 'Forward'
            lServo.turn(3)
            rServo.turn(-3)
            csock.send('move forward')
    elif commend=='s':
            print 'Backward'
            lServo.turn(-3)
            rServo.turn(3)
            csock.send('move backward')
    elif commend=='a':
            print 'Left'
            csock.send('turn left')
            #using dc motor class
    elif commend=='d':
            print 'Right'
            csock.send('turn right')
            #using dc motor class
    elif commend=='q':
            print 'Stop'
            lServo.turn(0)
            rServo.turn(0)
            csock.send('stop')
s.close()
GPIO.cleanup()