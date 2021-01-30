import robot
import threading
import RPi.GPIO as GPIO
import picamera
import Tcpnetworks
with picamera.PiCamera() as camera:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    camera.resolution = (640, 480)
    client = Tcpnetworks.Tcpnetworks()
    client.Client('192.168.137.127', 8080)
    slave=robot.Slave() #initial
    client.Blocking(0)
        
    ### This code can be increase Camera horizational stable 
    #if you write pull code plese delete 
    #try:
        #sender = threading.Thread(target=slave.to2master, args=(slave.AHRS.data))  #Threding object gen...
        #sender = threading.Thread(target=Send, args=(client.connection,))
        #sender.start() #Threading start.....
    
while True:       
#for i in range(1,100):
        from_server = client.ReceiveStr2()
        slave.AHRS.read()  #threading doesn't share variables
        AHRS_data=slave.AHRS.data
        print(AHRS_data)
        #client.SendStr(AHRS_data)
        yaw=slave.AHRS.splData[2]
        from_server = client.ReceiveStr2()
        client.SendStr(from_server + AHRS_data)
        print (from_server + data)
        command = ""
        if from_server==True:
            command = from_server
            camera.start_preview()
            camera.start_recording('cam_data.h264')
            camera.wait_recording(5)
        
        if  command==True:
            if command=='d':
                yaw=yaw+10
                slave.horCAMstable(yaw)
                AHRS_data=slave.AHRS.data
                client.SendStr(AHRS_data)
                #raspivid -o video.h264
                command = client.ReceiveStr2()
            elif command=='a':
                yaw=yaw-10
                slave.horCAMstable(yaw)
                AHRS_data=slave.AHRS.data
                client.SendStr(AHRS_data)
                #raspivid -o video.h264
                command = client.ReceiveStr2()
            elif command=='p':
                camera.stop_recording()
                camera.stop_preview()
                #ffmpeg -r 30 -i video.h264 vcodec copy video.mkv
            else:
                slave.AHRS.read()  #threading doesn't share variables
                AHRS_data=slave.AHRS.data
                client.SendStr(AHRS_data)
                slave.horCAMstable(yaw)
  
slave.stop()
GPIO.cleanup()