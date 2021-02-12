import time
import robot
import threading
import Tcpnetworks
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

server = Tcpnetworks.Tcpnetworks()
server.Serve('0.0.0.0', 8080)
go = robot.Master()
sender = threading.Thread(target=server.Send_th, args=(server.connection,))
#try : 
fout2 = open('ahrs.txt', 'w')
send_message = 'start'
server.SendStr(send_message) #syncronize with slave pi when slave pi receive "start"message start recording AHRS & camera
sender.start()
command=""
while True:
    AHRS_data = server.ReceiveStr2() # receive AHRS_data
    AHRS_data=str(AHRS_data)
    fout2.write(AHRS_data)
    print(AHRS_data)
    go.gyro.gyro_out()
    go.gyro.filewrite()
    command = AHRS_data[0] # clinet to server command

    if command == 'w' :
        for i in range(1,300):
            go.move(5)
            time.sleep(0.01)

    elif command == 's':
        for i in range(1,300):
            go.move(-5)
            time.sleep(0.01)

    elif command == 'a':
        print("turn Left")

    elif command=="d":
        print("turn Right")

    elif command == 'show': 
        for i in range(1,1000):
            go.move(5)
            time.sleep(0.01)
        go.move(0)
        for i in range(1,1000):
            go.move(-5)
            time.sleep(0.01)
        go.move(0)
        send_command = 'd'
        sever.SendStr(send_command)
        time.sleep(3000)
        send_command = 'a'
        sever.SendStr(send_command)
        time.sleep(3000)

    elif command=="p"
        print("Camera Recording pause")

    else :
        print ("wrong command")
#except:
go.stop()
GPIO.cleanup()