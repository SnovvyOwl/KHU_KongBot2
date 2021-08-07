#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<random>
#define D2R 0.017453293
#define R2D 57.295779513
#define SIGMA_V 10 //Measurement Noise
#define T 0.1
using namespace std;
using namespace cv;
/*
ms=0.51
mi=1.00
mp=0.36*2
Rs=0.15
Js=3.89*10**-3
Ji=4.42*10**-3
Jp=3.4*10**-4*2
Jr=0.024072249572891516
Jt=1.78703861879045e-003
g=9.81 
lp=0.1
rpin=0.0175
mt=0.38197
Mr=2.64589817207665;
a=6.382e-002
Rcm=0.06436548193923931
*/
default_random_engine generator;
class Pendulum{
    private:
        int shellTheta[3]={0,0,0};
        double pendulumTheta[4]={0.0,0.0,0.0,0.0};
        int motorinput[4]={0,0,0,0};
    public:
        Pendulum(float state){
            pendulumTheta[0]=state;//Current
            pendulumTheta[1]=state;//K-1
            pendulumTheta[2]=state;//K-2
            pendulumTheta[3]=state;//K-3
        }
        void clear(){
            shellTheta[0]=0;//Current
            shellTheta[1]=0;//K-1
            shellTheta[2]=0;//K-2
            pendulumTheta[0]=0;
            pendulumTheta[1]=0;
            pendulumTheta[2]=0;
            pendulumTheta[3]=0;
            motorinput[0]=0;
            motorinput[1]=0;
            motorinput[2]=0;
            motorinput[3]=0;

        }
        void calTheta(int input){
            /*
            //Pendulum Transfer Funtion  shell-> PENDULUMS
                         452.2 s + 5781                                   theta_p_real
                --------------------------------                =    --------------------------------
                    s^3 + 38.85 s^2 + 851.1 s + 5774                        theta_p_input
            
            
                0.3199 z^3 + 0.5695 z^2 + 0.1791 z - 0.07042 
                --------------------------------------------      
                 z^3 - 0.1121 z^2 + 0.1891 z - 0.08002                 

                //dt = 0.1
            */
            pendulumTheta[3]=pendulumTheta[2];
            pendulumTheta[2]=pendulumTheta[1];
            pendulumTheta[1]=pendulumTheta[0];
            motorinput[3]=shellTheta[2];
            motorinput[2]=shellTheta[1];
            motorinput[1]=shellTheta[0];
            motorinput[0]=(int)input;            
            pendulumTheta[0]= 0.1121*pendulumTheta[1]- 0.1891*pendulumTheta[2]+0.08002*pendulumTheta[3]+0.3199*motorinput[0]+ 0.5695*motorinput[1] + 0.1791 *motorinput[2] - 0.07042*motorinput[3]; 
            //pendulumTheta[0]=pendulumTheta[0]*D2R;
        }
       
        double getTheta(){
            return pendulumTheta[0];
        }
        double getVel(){
            return (pendulumTheta[0]-pendulumTheta[1])/T;
        }
        int motor(double gain){
            //INPUT PENDULUMS DEG 2 SERVO motor INPUT
            calTheta(gain);
            double input=0;
            if (pendulumTheta[0]>1.57079632679){  
                input=90.0;
            }
            else if(pendulumTheta[0]<-1.57079632679){
                input=-90.0;
            }
            else{
                input=pendulumTheta[0]*R2D;
            }
            return floor(1500+input*8.888889+0.5);
        }
};

class Shell{
    private:
        //KF FILTER VARIABLE
        Mat F=(Mat_<double>(4,4)<<1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
        Mat H=(Mat_<double>(1,4)<<0,1,0,0);
        Mat X=(Mat_<double>(4,1)<<0,0,0,0);
        Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
        Mat P=(Mat_<double>(4,4)<<1.0, 0.0, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.0,0.0,0.0,0.0,1.0);
        Mat I=(Mat_<double>(4,4)<<1.0, 0.0, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.0,0.0,0.0,0.0,1.0);
        Mat P_hat = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
        //Mat Q=(Mat_<double>(4,4)<<0.0001/24,0.001/6,0.001/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1); //SYSYTEM NOISE Need2Change
        Mat Q=(Mat_<double>(4,4)<<pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); 
        double R= SIGMA_V*SIGMA_V;
        Mat U=(Mat_<double>(1,1)<<0);
        Matx44d Qf;
        Mat Z_hat;
        Mat Z;
        Mat K;
        Mat S;
        //SHELL SPEED PID GAIN
        float kp=20;
        float ki=10;
        float kd=5;
        float pTerm=0;
        float iTerm=0;
        float dTerm=0;
        double shellTheta[2]={0,0};
        //double shellSpeed[3]={0,0,0};
        double shellSpeed=0;
        double penInput[3]={0,0,0};
        float errShellVel[3]={0.0,0.0,0.0};
        float preDesireVel=0;
        double gain[3]={0.0,0.0,0.0};
        normal_distribution<double>dist_W;//SYSTEM NOISE
        normal_distribution<double>dist_V;//SENSOR NOISE
        double W=0;
        double V=0;
    public:
        Shell(normal_distribution<double>w,normal_distribution<double> v){
            dist_W=w;
            dist_V=v;
            W=dist_W(generator);
            X_hat.copySize(P.mul(W).diag());
            X_hat=X+P.mul(W).diag();
            P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
            Qf=Q;
        }
        void clear(){
            shellTheta[0]=0;
            shellTheta[1]=0;
            //X.clear();
        }
        void calSystem(double ThetaP){
            //modeling
            X_hat.at<double>(0)=X.at<double>(0)+X.at<double>(1)*T;
            X_hat.at<double>(1)=X.at<double>(1)-U.at<double>(0)*T;
            X_hat.at<double>(2)=X.at<double>(2)+X.at<double>(3)*T;
            X_hat.at<double>(3)=X.at<double>(3)-1037.647058824*sin(ThetaP)*T+1470.588235294*U.at<double>(0)*T;
            //Jacobian
            double cosThetaP=cos(ThetaP);
            F=(Mat_<double>(4,4)<<1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0, 0.0,0.0,1 - 5.1882353*cosThetaP,0.1 - 0.1729412*cosThetaP,0.0,0.0, 179.451903*cosThetaP*cosThetaP-103.764706*cosThetaP,1 - 5.188235*cosThetaP);
        }
        void calAngularVelocity(float AHRStheta,float encodertheta,Pendulum &pen){
            //Calculate AngularVelocity 
            shellTheta[1]=shellTheta[0];
            shellTheta[0]= AHRStheta-encodertheta;
            //shellSpeed[2]=shellSpeed[1];
            //shellSpeed[1]=shellSpeed[0];
            shellSpeed=(shellTheta[0]-shellTheta[1])/T;
            X=(Mat_<double>(4,1)<<shellTheta[0],shellSpeed,pen.getTheta(),pen.getVel());
            EKF(pen);
        }
        double shell2pen(){
            /*
            -(Js+Rs^2*mi+Rs^2*mp+R^2*ms)S^2                    -0.054064999999999995*S^2
            -------------------------------         =   -------------------------------
            jp*S^2+mp*lp*g                                0.00068*S^2+ 0.70632

            -22.11 z^2 + 44.21 z - 22.11                theta_p
            ----------------------------   =        -------------
                z^2 + 0.8879 z + 1                      s*theta_s
            dt = 0.1
            */
            penInput[2]=penInput[1];
            penInput[1]=penInput[0];
            penInput[0]=-22.11*gain[0]+44.21*gain[1]-22.11*gain[2]-0.8879*penInput[1]-penInput[2];
            return penInput[0];
        }
        void EKF(Pendulum &pen){
            W=dist_W(generator);
            V= dist_V(generator);
            //==================
            // TIME UPDATE
            // =================
            calSystem(pen.getTheta()*D2R);   
            P_hat=F*P*F.t()+Qf; //Qf가 K에 대한 함수가 아닌가?
            //==================
            //MESUREMENT UPDATE
            //=================
            S=H*P_hat*H.t()+R;//R가 K에 대한 함수가 아닌가?
            K=P_hat*H.t()*S.inv();
            Z_hat=H*X_hat;
            X=X_hat+K*(Z-Z_hat);
            P=(I-K*H)*P_hat;
            shellSpeed=X.at<double>(1);
        }
        int speedControl(Pendulum& pen,float desireVel){
            //PID CONTROLLER FOR SHELL SPEED
            if (desireVel!=preDesireVel){
                PIDtermClear();
            }
            gain[2]=gain[1];
            gain[1]=gain[0];
            preDesireVel=desireVel;
            errShellVel[1]=errShellVel[0];
            errShellVel[0]=desireVel-X.at<double>(1);
            pTerm=kp*errShellVel[0];
            iTerm+=ki*errShellVel[0]*T;
            dTerm=kd*(errShellVel[0]-errShellVel[1])/T;
            gain[0]=pTerm+iTerm-dTerm;
            return pen.motor(shell2pen());
        }
        void PIDtermClear(){
            pTerm=0;
            iTerm=0;
            dTerm=0;
        }
};

class Tilt{
    private:
        float tiltTheta[4]={0.0,0.0,0.0,0.0};
        float RCM=0;
        float err[2]={0.0,0.0};
         //SHELL SPEED PID GAIN
        float kp=20;
        float ki=10;
        float kd=5;
        float pTerm=0;
        float iTerm=0;
        float dTerm=0;
        float iduTheta[2]={0,0};
        float desireTheta[2]={0.0,0.0};
        float preDesireTheta=0;
        Mat F=(Mat_<double>(4,4)<<1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
        Mat H=(Mat_<double>(1,4)<<0,1,0,0);
        Mat X=(Mat_<double>(4,1)<<0,0,0,0);
        Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
        Mat P=(Mat_<double>(4,4)<<1.0, 0.0, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.0,0.0,0.0,0.0,1.0);
        Mat I=(Mat_<double>(4,4)<<1.0, 0.0, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.0,0.0,0.0,0.0,1.0);
        Mat P_hat = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
        //Mat Q=(Mat_<double>(4,4)<<0.0001/24,0.001/6,0.001/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1); //SYSYTEM NOISE Need2Change
        Mat Q=(Mat_<double>(4,4)<<pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); 
        double R= SIGMA_V*SIGMA_V;
        Mat U=(Mat_<double>(1,1)<<0);
        Matx44d Qf;
        Mat Z_hat;
        Mat Z;
        Mat K;
        Mat S;
    public:
        Tilt tilt(){}      
        void clear(){
            //tiltTheta[4]={0.0,0.0,0.0,0.0};
            RCM=0;
            //err[2]={0.0,0.0};
        }
        void setRCM(Pendulum &pen){
            RCM=pen.massCenter();
        }
        void idu2tilt(){

        }
        void calSystem(float AHRStheta) {

            
            //modeling
            X_hat.at<double>(0)=X.at<double>(0)+X.at<double>(1)*T;
            X_hat.at<double>(1)=X.at<double>(1)-U.at<double>(0)*T;
            X_hat.at<double>(2)=X.at<double>(2)+X.at<double>(3)*T;
            X_hat.at<double>(3)=X.at<double>(3)-1037.647058824*sin(ThetaP)*T+1470.588235294*U.at<double>(0)*T;
            //Jacobian

            // F =
            //x1=th_t
            //x3=th_x
            // [1,  0.1, 0.000858375*cos(x3), 0]
            // [0, 1, 0.0171675*cos(x3), 0.000858375*cos(x3)]
            // [0.000129285*x3*sin(x1), 0, 1 - 0.0083534*cos(x3) - 1.29285*e^-4cos(x1), 0.1]
            // [0.0025857*x3*sin(x1), 0.000129285*x3*sin(x1), -0.0025857*cos(x1) -0.1670687*cos(x3), 1 - 0.0083534*cos(x3) -1.29285*e^-4*cos(x1)]
            double cosThetaX=cos(AHRStheta*D2R);
            double cosThetaT=cos(tiltTheta[0]*D2R);
            double sinThetaT=sin(tiltTheta[0]*D2R);
            F=(Mat_<double>(4,4)<<1,  0.1, 0.000858375*cosThetaX, 0,0, 1, 0.0171675*cosThetaX, 0.000858375*cosThetaX,0.000129285*AHRStheta*D2R*sinThetaT, 0, 1 - 0.0083534*cosThetaX - 0.000129285*cosThetaT, 0.1,0.0025857*AHRStheta*D2R*sinThetaT, 0.000129285*AHRStheta*D2R*sinThetaT, -0.0025857*cosThetaT -0.1670687*cosThetaX, 1 - 0.0083534*cosThetaX -0.000129285*cosThetaT);
        void EKF(){
            
        }
        int motor(){
            int motorInput=0;
            if (tiltTheta[0]>120){  
                tiltTheta[0]=120.0;
            }
            else if(tiltTheta[0]<-120){
                tiltTheta[0]=-120.0;
            }
            motorInput= floor(1500+tiltTheta[0]* 6.66667+0.5);
            return motorInput;
        }
        int rollControl(Pendulum &pen,float desireTheta){
             if (desireTheta!=preDesireTheta){
                PIDtermClear();
            }
            preDesireTheta=desireTheta;
            idu2tilt();
            return motor();
        }
        void PIDtermClear(){
            pTerm=0;
            iTerm=0;
            dTerm=0;
        }
};

class Idu{
    private: 
        float iduTheta[4]={0.0,0.0,0.0,0.0}; //array resized is reqired
        //IDU STABLE PID GAIN
        float kp=20;
        float ki=10;
        float kd=5;
        float pTerm=0;
        float iTerm=0;
        float dTerm=0;
        double shellTheta[2]={0,0};
        float err[2]={0.0,0.0};
        float gain=0;
    public:
        Idu idu(){}
        int stableControl(float pitch){
            err[1]=err[0];
            err[0]=-pitch;
            pTerm=kp*err[0];
            iTerm+=ki*err[0]*T;
            dTerm=kd*(err[0]-err[1])/T;
            gain=pTerm+iTerm-dTerm;
            return gain;
        }
        int motor(){
            //????
            if (gain>90){  
                gain=90.0;
            }
            else if(gain<-90){
                gain=-90.0;  // anguler Velocity  cal   max speed ;;;
            }
            return floor(1500+gain*8.888889+0.5);
        }
        void PIDtermClear(){
            pTerm=0;
            iTerm=0;
            dTerm=0;
        }
};
