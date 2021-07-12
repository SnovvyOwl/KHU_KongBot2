#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<random>
#define SIGMA_V 10 //Measurement Noise
#define T 0.1
using namespace std;
using namespace cv;
default_random_engine generator;
//normal_distribution<double> dist_W(0,1.0);
//normal_distribution<double> dist_V(0,1.0);
class Pendulum{
    private:
        int shellTheta[4]={0,0,0,0};
        float pendulumTheta[4]={0.0,0.0,0.0,0.0};
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
            //Pendulum Transfer Funtion  
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
        float getPenTheta(){
            return pendulumTheta[0];
        }
        float getPenVel(){
            return (pendulumTheta[0]-pendulumTheta[1])/T;
        }
        int pen2motor(){
            int motor=0;
            if (pendulumTheta[0]>90){  
                pendulumTheta[0]=90.0;
            }
            else if(pendulumTheta[0]<-90){
                pendulumTheta[0]=-90.0;
            }
            motor= floor(1500+pendulumTheta[0]*8.888889+0.5);
            return motor;
        }
};

class Shell{
    private:
        //KF FILTER VARIABLE
        Mat F=(Mat_<double>(4,4)<<1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
        Mat G=(Mat_<double>(4,1)<<-0.0244, -0.4888,7.3529 ,147.0588);
        Mat H=(Mat_<double>(1,4)<<0,1,0,0);
        Mat X=(Mat_<double>(4,1)<<0,0,0,0);
        Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
        Mat P=(Mat_<double>(4,4)<<1.0, 0.0, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.0,0.0,0.0,0.0,1.0);
        Mat X_bar;
        Mat p_bar = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
        Mat p_hat = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
        //KF Noise
        Matx44d Q(0.0001/24,0.001/6,0.001/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1); //SYSYTEM NOISE Need2Change
        //Matx44d Q(pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); 
        double R= SIGMA_V*SIGMA_V;
        Mat U=(Mat_<double>(1,1)<<0);
        Matx44d Qf;
        Mat z_hat;
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
        float shellTheta[2]={0,0};
        float errShellVel[2]={0.0,0.0};
        float preDesireVel=0;
        float gain=0;
        normal_distribution<double>dist_W;//SYSTEM NOISE
        normal_distribution<double>dist_V;//SENSOR NOISE
        double W=0;
        double V=0;
    public:
        //cv::sqrt(P,P);
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
        void calJacobian(){
            
        }
        void calAngularVelocity(float AHRStheta,float encodertheta,Pendulum &pen){
            shellTheta[1]=shellTheta[0];
            shellTheta[0]= AHRStheta-encodertheta;
            float shellvel=(shellTheta[0]-shellTheta[1])/T;
            X[0][0]=shellTheta;
            X[0][1]=shellvel;
            X[0][2]=pen.getPenTheta();
            X[0][3]=pen.getPenVel();
            KF();
            //return X[0][1];
            //return X_hat[0][1];
        }
        void KF(){
            V= dist_V(generator);
            Z=H*X+sqrt(R)*V; //MESUREMENT MODEL
            //==================
            //MESUREMENT UPDATE
            //=================
            z_hat=H*X_bar;
            S=H*p_bar*H.t()+R;
            p_hat=p_bar-p_bar*H.t()*S.inv()*H*p_bar;
            K=p_bar*H.t()*S.inv();
            X_hat=X_bar+K*(Z-z_hat);
            //==================
            // TIME UPDATE
            // =================
            calJacobian();// Change F
            X_bar=F*X_hat+G*U;
            p_bar=F*p_hat*F.t()+Qf;
            //==================
            //System Dynamics
            //==================
            cv::sqrt(Q,Q);
            W=dist_W(generator);
            X=F*X+G*U+Q.diag()*W;
        }
        int speedControl(Pendulum &pen,float desireVel){
            //PID CONTROLLER
            if (desireVel!=preDesireVel){
                PIDtermClear();
            }
            preDesireVel=desireVel;
            errShellVel[1]=errShellVel[0];
            errShellVel[0]=desireVel-(int)X[0][1];
            pTerm=kp*errShellVel[0];
            iTerm+=ki*errShellVel[0]*T;
            dTerm=kd*(errShellVel[0]-errShellVel[1])/T;
            gain=pTerm+iTerm-dTerm;
            pen.shell2pen(gain);
            return pen.pen2motor();
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
        //생성자
        Tilt tilt(){
                dTerm=0;
        }
        void pentheta2Rcm(Pendulum &pen){
            pen.getPenTheta();
            //RCM=nuber
        }
        void clear(){
            //tiltTheta[4]={0.0,0.0,0.0,0.0};
            RCM=0;
            //err[2]={0.0,0.0};
        }
        void idu2tilt(){

        }
        void calJacobian(){

        }
        void KF(){

        }
        int tilt2motor(){
            int motor=0;
            if (tiltTheta[0]>90){  
                tiltTheta[0]=90.0;
            }
            else if(tiltTheta[0]<-90){
                tiltTheta[0]=-90.0;
            }
            motor= floor(1500+tiltTheta[0]*8.888889+0.5);
            return motor;
        }
        int rollControl(Pendulum &pen,float desireTheta){
             if (desireTheta!=preDesireTheta){
                PIDtermClear();
            }
            preDesireTheta=desireTheta;
            pentheta2Rcm(Pendulum &pen);
            idu2tilt();
            return tilt2motor();
        }
        void PIDtermClear(){
            pTerm=0;
            iTerm=0;
            dTerm=0;
        }
};

class Idu{
    private: 
        float iduTheta[4]={0.0,0.0,0.0,0.0};
        float err[2]={0.0,0.0};
    public:
        //생성자
        Idu idu(){
            err[0]=0;
        }
        int control(){
            return 0;
        }
        int idu2motor(){
            int motor=0;
            if (iduTheta[0]>90){  
                iduTheta[0]=90.0;
            }
            else if(iduTheta[0]<-90){
                iduTheta[0]=-90.0;
            }
            motor= floor(1500+iduTheta[0]*8.888889+0.5);
            return motor;
        }
};
