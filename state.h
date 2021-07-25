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
g=9.81
lp=0.1
*/
default_random_engine generator;
class Pendulum{
    private:
        int shellTheta[4]={0,0,0,0};
        double pendulumTheta[4]={0.0,0.0,0.0,0.0};
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
            shellTheta[3]=0;//K-3
            pendulumTheta[0]=0;
            pendulumTheta[1]=0;
            pendulumTheta[2]=0;
            pendulumTheta[3]=0;
        }
        int shell2pen(float curshellTheta){
            //Pendulum Transfer Funtion  shell-> PENDULUMS
            /*             452.2 s + 5781                                   theta_p
                --------------------------------                =    --------------------------------
                    s^3 + 38.85 s^2 + 851.1 s + 5774                        theta_s
            
            
                0.3199 z^3 + 0.5695 z^2 + 0.1791 z - 0.07042 
                --------------------------------------------      
                 z^3 - 0.1121 z^2 + 0.1891 z - 0.08002                 

                //dt = 0.1
            */
            pendulumTheta[3]=pendulumTheta[2];
            pendulumTheta[2]=pendulumTheta[1];
            pendulumTheta[1]=pendulumTheta[0];
            shellTheta[3]=shellTheta[2];
            shellTheta[2]=shellTheta[1];
            shellTheta[1]=shellTheta[0];
            shellTheta[0]=(int)curshellTheta;            
            pendulumTheta[0]= 0.1121*pendulumTheta[1]- 0.1891*pendulumTheta[2]+0.08002*pendulumTheta[3]+0.3199*shellTheta[0]+ 0.5695*shellTheta[1] + 0.1791 *shellTheta[2] - 0.07042*shellTheta[3]; 
            return pendulumTheta[0];
        }
        double getTheta(){
            return pendulumTheta[0];
        }
        double getVel(){
            return (pendulumTheta[0]-pendulumTheta[1])/T;
        }
        int motor(){
            //INPUT PENDULUMS DEG 2 SERVO motor INPUT
           
            if (pendulumTheta[0]>90){  
                pendulumTheta[0]=90.0;
            }
            else if(pendulumTheta[0]<-90){
                pendulumTheta[0]=-90.0;
            }
            return floor(1500+pendulumTheta[0]*8.888889+0.5);
        }
        double pen2Rcm(){
            //calculate RCM;
            return (−0.001927186-0.072*cos(pendulumTheta[0])+0.382*0.063823319)/2.4614;
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
        float errShellVel[2]={0.0,0.0};
        float preDesireVel=0;
        float gain=0;
        normal_distribution<double>dist_W;//SYSTEM NOISE
        normal_distribution<double>dist_V;//SENSOR NOISE
        double W=0;
        double V=0;
    public:
        Shell(normal_distribution<double>w,normal_distribution<double> v){
            dist_W=w;
            dist_V=v;
            W=dist_W(generator);
            X_bar.copySize(P.mul(W).diag());
            X_bar=X+P.mul(W).diag();
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
            X_bar.at<double>(0)=X.at<double>(0)+X.at<double>(1)*T;
            X_bar.at<double>(1)=X.at<double>(1)-U.at<double>(0)*T;
            X_bar.at<double>(2)=X.at<double>(2)+X.at<double>(3)*T;
            X_bar.at<double>(3)=X.at<double>(3)-1037.647058824*sin(ThetaP)*T+1470.588235294*U.at<double>(0)*T;
            //Jacobian
            cosThetaP=cos(ThetaP);
            F=(Mat_<double>(4,4)<<1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0, 0.0,0.0,1 - 5.1882353*cosThetaP,0.1 - 0.1729412*cosThetaP,0.0,0.0, 179.451903*cosThetaP*cosThetaP-103.764706*cosThetaP,1 - 5.188235*cosThetaP);
        }
        void calAngularVelocity(float AHRStheta,float encodertheta,Pendulum &pen){
            //Calculate AngularVelocity 
            shellTheta[1]=shellTheta[0];
            shellTheta[0]= AHRStheta-encodertheta;
            double shellvel=(shellTheta[0]-shellTheta[1])/T;
            X=(Mat_<double>(4,1)<<shellTheta[0],shellvel,pen.getTheta(),pen.getVel());
            EKF();
            //return X(0,1);
            //return X_hat(0,1);
        }
        void EKF(Pendulum &pen){
            W=dist_W(generator);
            V= dist_V(generator);
            //==================
            // TIME UPDATE
            // =================
            calSystem(pen.getTheta()*D2R)   
            P_hat=F*P*F.t()+Qf; //Qf가 K에 대한 함수가 아닌가?
            //==================
            //MESUREMENT UPDATE
            //=================
            S=H*P_hat*H.t()+R;//R가 K에 대한 함수가 아닌가?
            K=P_hat*H.t()*S.inv();
            Z_hat=H*X_hat;
            X=X_hat+K*(Z-Z_hat);
            P=(I-K*H)*P_hat;
        }
        int speedControl(Pendulum& pen,float desireVel){
            //PID CONTROLLER FOR SHELL SPEED
            if (desireVel!=preDesireVel){
                PIDtermClear();
            }
            preDesireVel=desireVel;
            errShellVel[1]=errShellVel[0];
            errShellVel[0]=desireVel-X.at<double>(1);
            pTerm=kp*errShellVel[0];
            iTerm+=ki*errShellVel[0]*T;
            dTerm=kd*(errShellVel[0]-errShellVel[1])/T;
            gain=pTerm+iTerm-dTerm;
            pen.shell2pen(gain);
            return pen.motor();
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
    public:
        Tilt tilt(){}      
        void clear(){
            //tiltTheta[4]={0.0,0.0,0.0,0.0};
            RCM=0;
            //err[2]={0.0,0.0};
        }
        void idu2tilt(){

        }
        void calJacobian() {
            
        }
        void KF(){
            
        }
        int motor(){
            int motorinput=0;
            if (tiltTheta[0]>120){  
                tiltTheta[0]=120.0;
            }
            else if(tiltTheta[0]<-120){
                tiltTheta[0]=-120.0;
            }
            motorinput= floor(1500+tiltTheta[0]* 6.66667+0.5);
            return motorinput;
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
