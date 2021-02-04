import motor
import sensor

class Master:
    def __init__(self):
        self.gyro=sensor.Gyro()
        self.PLServo=motor.Servo(14,6.77)
        self.PRServo=motor.Servo(12,6.77)
        self.CamServo=motor.Servo(19,6.77) #NO PWM CHANNEL
        self.from_client=[0,0,0,0,0,0,0,0,0]
        self.CprevEr=0
        self.CsumEr=0
        self.PprevEr=0
        self.PsumEr=0

    def move(self,ANGLEVEL):
        # PID GAIN
        kp=20
        ki=10
        kd=5
        target=ANGLEVEL
        self.gyro.read()
        AngVYER=target-self.gyro.yout
        controlgain+=(AngVYER*kp)+(self.PprevEr*kd)+(self.PsumEr*ki)
        ##We need theta' derivative to PWM
        controlgain=max(min(12,controlgain),0)
        self.PLServo.turn(controlgain)
        self.PRServo.turn(12-controlgain)
        self.stableVision()
        self.PprevEr=AngVYER
        self.PsumEr+=AngVYER
  
    def stableVision(self,client):
         # PID GAIN
        kp=20
        ki=10
        kd=5
        self.gyro.read()
        Er=self.Gyro.roty-client # maybe we need sampling time....
        controlgain+=(Er*kp)+(self.CprevEr*kd)+(self.CsumEr*ki)
        controlgain=max(min(12,controlgain),0)
        self.CamServo.turn(controlgain)
        self.CprevEr=Er
        self.CsumEr+=Er
    
    def stop(self):
        PLServo.stop()
        PRServo.stop()
        CamServo.stop()
    
class Slave:
    def __init__(self):
        self.wheel=motor.DCmotor(12,19)
        self.AHRS=sensor.serialConnect("/dev/ttyUSB0",115200)
        self.prevEr=0
        self.sumEr=0
        self.controlgain=0
    
    def turn(self,Dir,INPUT):
        self.prevEr=0
        self.sumEr=0  
        if Dir == 'TR' :
            motor.DCmotor.CW(INPUT)
        elif Dir == 'TL' :
            motor.DCmotor.CCW(INPUT)
    
    def horCAMstable(self,initYAW):
         # PID GAIN
        kp=20
        ki=10
        kd=5
        #init error
        self.AHRS.read()
        YawER=initYAW-float(self.AHRS.splData[2])
        self.controlgain+=(YawER*kp)+((YawER-self.prevEr)*kd)+(self.sumEr*ki)
        if self.controlgain>0:
            input=max(min(100,self.controlgain),0)
            self.wheel.CW(input)
        else:
            input=-self.controlgain
            input=max(min(100,input),0)
            self.wheel.CCW(input)
        self.prevEr=YawER
        self.sumEr+=YawER
    
    def stop(self):
        self.wheel.stop()