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
class Pendulum {
private:
    int shellTheta[3] = { 0,0,0 };//DEGREE
    double pendulumTheta[4] = { 0.0,0.0,0.0,0.0 };//DEG
    int motorinput[4] = { 0,0,0,0 };//PULSE WIDTH 700~2300
public:
    Pendulum(){}
    void clear() {
        shellTheta[0] = 0;//Current
        shellTheta[1] = 0;//K-1
        shellTheta[2] = 0;//K-2
        pendulumTheta[0] = 0;
        pendulumTheta[1] = 0;
        pendulumTheta[2] = 0;
        pendulumTheta[3] = 0;
        motorinput[0] = 0;
        motorinput[1] = 0;
        motorinput[2] = 0;
        motorinput[3] = 0;
    }
    void calTheta() {
        /* OBSERVER
        #############################################################################
            Pendulum Transfer Funtion  input(deg)-> Pendulums Theta (deg)
        #############################################################################
                     452.2 s + 5781                                   theta_p_real
            --------------------------------                =    --------------------------------
                s^3 + 38.85 s^2 + 851.1 s + 5774                        theta_p_input


            0.3199 z^3 + 0.5695 z^2 + 0.1791 z - 0.07042
            --------------------------------------------
             z^3 - 0.1121 z^2 + 0.1891 z - 0.08002
            dt = 0.1
        */
        pendulumTheta[3] = pendulumTheta[2];
        pendulumTheta[2] = pendulumTheta[1];
        pendulumTheta[1] = pendulumTheta[0];
        pendulumTheta[0] = 0.1121 * pendulumTheta[1] - 0.1891 * pendulumTheta[2] + 0.08002 * pendulumTheta[3] + 0.3199 * motorinput[0] + 0.5695 * motorinput[1] + 0.1791 * motorinput[2] - 0.07042 * motorinput[3];
    }

    double getTheta() {
        return pendulumTheta[0];
    }
    double getVel() {
        return (pendulumTheta[0] - pendulumTheta[1]) / T;
    }
    int motor(double gain) {
        //#####################################
        //INPUT GAIN -> SERVO motor INPUT
        //arg    : gain {PID GAIN FOR SHELL SPEED CONTROL}
        //return : motorinput[0]  {PENDULUMS SERVO MOTOR INPUT}
        //#####################################
        motorinput[3] = motorinput[2];
        motorinput[2] = motorinput[1];
        motorinput[1] = motorinput[0];
        if (gain > 1.57079632679) {
            gain = 90.0;
        }
        else if (gain < -1.57079632679) {
            gain = -90.0;
        }
        else {
            gain = gain * R2D;
        }
        motorinput[0] = gain;
        return floor(1500 + gain * 8.888889 + 0.5);
    }
};

class Shell {
private:
    //EKF FILTER VARIABLE
    Mat F = (Mat_<double>(4, 4) << 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, -0.00009637185, 1);
    Mat H = (Mat_<double>(1, 4) << 0, 1, 0, 0);
    Mat X = (Mat_<double>(4, 1) << 0, 0, 0, 0);
    Mat X_hat = (Mat_<double>(4, 1) << 0, 0, 0, 0);
    Mat P = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    Mat I = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    Mat P_hat = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1) * 1000;
    Mat Q = (Mat_<double>(4, 4) << pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T); //SYSTEM COVARIANCE
    double R = SIGMA_V * SIGMA_V; //Measurement COVARIANCE
    Mat U = (Mat_<double>(1, 1) << 0);
    Mat Z_hat;
    Mat Z;
    Mat K;
    Mat S;
    normal_distribution<double>dist_W;//SYSTEM NOISE
    normal_distribution<double>dist_V;//SENSOR NOISE
    double W = 0;
    double V = 0;
    //SHELL SPEED PID GAIN
    float kp = 20;
    float ki = 10;
    float kd = 5;
    float pTerm = 0;
    float iTerm = 0;
    float dTerm = 0;
    double shellTheta[2] = { 0,0 };
    double shellSpeed[2] = { 0,0 };
    double penInput[3] = { 0,0,0 };
    float errShellVel[2] = { 0.0,0.0 };
    float preDesireVel = 0;//rad
    double gain[3] = { 0.0,0.0,0.0 };

public:
    Shell() {
        X_hat.copySize(P.mul(W).diag());
        X_hat = X + P.mul(W).diag();
        P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1) * 1000;
    }
    void setDist(normal_distribution<double>&w, normal_distribution<double> &v){
        dist_W = w;
        dist_V = v;
        W = dist_W(generator);
    }
    void clear() {
        shellTheta[0] = 0;
        shellTheta[1] = 0;
        //X.clear();
    }
    void calAngularVelocity(float AHRStheta, float encodertheta, double penTheta, double penVel) {
        //##################################################
        //Calculate AngularVelocity for Shell
        //arg: AHRStheta{AHRS read pitch angle[DEG]}, encodertheta {encoder theta[DEG]} , penTheta{Pendulum theta [DEG]} , penVel{pendulum Velcoty[DEG/s]}
        //##################################################
        shellTheta[1] = shellTheta[0];
        shellTheta[0] = AHRStheta - encodertheta;
        shellSpeed[1] = shellSpeed[0];
        shellSpeed[0] = (shellTheta[0] - shellTheta[1]) / T; //SENSOR READ SHELL VELOCITY
        //X=(Mat_<double>(4,1)<<shellTheta[0]*D2R,shellSpeed[0]*D2R,penTheta,penVel);
        Z = (Mat_<double > (1, 1) << shellSpeed[0] * D2R);
        EKF(penTheta * D2R);
    }
    void EKF(double penTheta) {
        //######################################
        //EXTENDED KALMAN FILTER
        // arg: penTheta {pendulum Theta [Rad]}
        // #####################################
        W = dist_W(generator);
        V = dist_V(generator);
        //==================
        // TIME UPDATE
        // =================
        calSystem(penTheta);
        P_hat = F * P * F.t() + Q;
        //==================
        //MESUREMENT UPDATE
        //=================
        S = H * P_hat * H.t() + R;
        K = P_hat * H.t() * S.inv();
        Z_hat = H * X_hat;
        X = X_hat + K * (Z - Z_hat);
        P = (I - K * H) * P_hat;
        shellSpeed[0] = X.at<double>(1) * R2D;
    }
    void calSystem(double penTheta) {
        //##############################################
        //Calculate F matrix For EKF
        //arg   : penTheta {Pendulums Current Theta [RAD]}
        //##############################################
        //modeling
        X_hat.at<double>(0) = X.at<double>(0) + X.at<double>(1) * T;
        X_hat.at<double>(1) = X.at<double>(1) - U.at<double>(0) * T;
        X_hat.at<double>(2) = X.at<double>(2) + X.at<double>(3) * T;
        X_hat.at<double>(3) = X.at<double>(3) - 1037.647058824 * sin(penTheta) * T + 1470.588235294 * U.at<double>(0) * T;
        //Jacobian
        double cosThetaP = cos(penTheta);
        F = (Mat_<double>(4, 4) << 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1 - 5.1882353 * cosThetaP, 0.1 - 0.1729412 * cosThetaP, 0.0, 0.0, 179.451903 * cosThetaP * cosThetaP - 103.764706 * cosThetaP, 1 - 5.188235 * cosThetaP);
    }

    double speedControl(float desireVel) {
        //#############################################
        //PID CONTROLLER FOR SHELL SPEED
        //arg : desireVel{desire Shell Velocity{Deg/s}}
        //return : shell2pen() {pendulum motor input Gain}
        //#############################################
        desireVel = desireVel * D2R;
        if (desireVel != preDesireVel) {
            PIDtermClear();
        }
        gain[2] = gain[1];
        gain[1] = gain[0];
        preDesireVel = desireVel;
        errShellVel[1] = errShellVel[0];
        errShellVel[0] = desireVel - X.at<double>(1);
        pTerm = kp * errShellVel[0];
        iTerm += ki * errShellVel[0] * T;
        dTerm = kd * (errShellVel[0] - errShellVel[1]) / T;
        gain[0] = pTerm + iTerm - dTerm;
        return  shell2pen();
        }
        double shell2pen() {
            /*
            ################################################################################
                Shell's Angular VElOCITY PID GAIN-> PENULUM THETA
                return : penINPUT[0] {pendulum motor input Gain}
            ################################################################################
            -(Js+Rs^2*mi+Rs^2*mp+R^2*ms)S^2                    -0.054064999999999995*S^2
            -------------------------------         =   -------------------------------
            jp*S^2+mp*lp*g                                0.00068*S^2+ 0.70632
            -22.11 z^2 + 44.21 z - 22.11                theta_p
            ----------------------------   =        -------------
                z^2 + 0.8879 z + 1                      s*theta_s
            dt = 0.1
            */
            penInput[2] = penInput[1];
            penInput[1] = penInput[0];
            penInput[0] = -22.11 * gain[0] + 44.21 * gain[1] - 22.11 * gain[2] - 0.8879 * penInput[1] - penInput[2];
            return penInput[0];
        }

        void PIDtermClear() {
            pTerm = 0;
            iTerm = 0;
            dTerm = 0;
            errShellVel[0] = 0;
            errShellVel[1] = 0;
            preDesireVel = 0;//rad
        }
    };

    class Tilt {
    private:
        float tiltTheta[4] = { 0.0,0.0,0.0,0.0 };//rad
        double iduTheta[2] = { 0.0,0.0 };//deg
        float RCM = 0;
        double gain[4] = { 0.0,0.0,0.0,0.0 };
        float errTiltAngle[2] = { 0.0,0.0 };
        int tiltInput[4] = { 0,0,0,0 };
        //SHELL SPEED PID GAIN
        float kp = 20;
        float ki = 10;
        float kd = 5;
        float pTerm = 0;
        float iTerm = 0;
        float dTerm = 0;
        float desireTheta = 0;
        float preDesireTheta = 0;
        Mat F = (Mat_<double>(4, 4) << 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, -0.00009637185, 1);
        Mat H = (Mat_<double>(1, 4) << 0, 0, 1, 0);
        Mat X = (Mat_<double>(4, 1) << 0, 0, 0, 0);
        Mat X_hat = (Mat_<double>(4, 1) << 0, 0, 0, 0);
        Mat P = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        Mat I = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        Mat P_hat = cv::Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1) * 1000;
        //Mat Q=(Mat_<double>(4,4)<<0.0001/24,0.001/6,0.001/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1,0.0001/24,0.001/6,0.01/2,0.1); //SYSYTEM NOISE Need2Change
        Mat Q = (Mat_<double>(4, 4) << pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T, pow(T, 4) / 24, pow(T, 3) / 6, pow(T, 2) / 2, T);
        double R = SIGMA_V * SIGMA_V;
        Mat U = (Mat_<double>(1, 1) << 0);
        Mat Z_hat;
        Mat Z;
        Mat K;
        Mat S;
        normal_distribution<double>dist_W;//SYSTEM NOISE
        normal_distribution<double>dist_V;//SENSOR NOISE
        double W = 0;
        double V = 0;
    public:
        Tilt() {}
        void setDist(normal_distribution<double>&w, normal_distribution<double> &v){
            dist_W = w;
            dist_V = v;
            W = dist_W(generator);
        }
        void clear() {
            tiltTheta[0]=0;
            tiltTheta[1]=0;
            tiltTheta[2]=0;
            tiltTheta[3]=0;
            RCM = 0;
            errTiltAngle[0] = 0;
            errTiltAngle[1] = 0;
        }
        void setRCM(double massCenter) {
            //############################################
            //set Center of mass of Robot
            //arg: massCenter{mass center of Robot{m}}
            //############################################
            RCM = massCenter;
        }
        void calRollAngle(float AHRSTheta) {
            //##################################################
            //Calculate Roll angle of Robot
            //arg: AHRStheta{AHRS read roll angle[DEG]}}
            //##################################################
            Z = (Mat_<double> (1,1) << AHRSTheta * D2R);
            EKF(AHRSTheta * D2R);
        }

        void EKF(float rollTheta) {
            //######################################
            //  EXTENDED KALMAN FILTER
            // arg: rollTheta {Robot Roll angle [Rad]}
            // #####################################
            W = dist_W(generator);
            V = dist_V(generator);
            //==================
            // TIME UPDATE
            // =================
            calSystem(rollTheta);
            P_hat = F * P * F.t() + Q;
            //==================
            //MESUREMENT UPDATE
            //=================
            S = H * P_hat * H.t() + R;
            K = P_hat * H.t() * S.inv();
            Z_hat = H * X_hat;
            X = X_hat + K * (Z - Z_hat);
            P = (I - K * H) * P_hat;
            iduTheta[0] = X.at<double>(2);
        }
        void calSystem(float rollTheta) {
            //##############################################
            //Calculate F matrix For EKF
            //arg   : AHRSheta {IDU ROLL theta [DEG]}
            //##############################################
            //modeling

            // X_HAT CAL

            //G?? ??
            //Jacobian
            //x1=th_t
            //x3=th_x
            //F =
            // [1,  0.1, 0.000858375*cos(x3), 0]
            // [0, 1, 0.0171675*cos(x3), 0.000858375*cos(x3)]
            // [0.000129285*x3*sin(x1), 0, 1 - 0.0083534*cos(x3) - 1.29285*e^-4cos(x1), 0.1]
            // [0.0025857*x3*sin(x1), 0.000129285*x3*sin(x1), -0.0025857*cos(x1) -0.1670687*cos(x3), 1 - 0.0083534*cos(x3) -1.29285*e^-4*cos(x1)]
            double cosThetaX = cos(rollTheta * D2R);
            double cosThetaT = cos(tiltTheta[0] * D2R);
            double sinThetaT = sin(tiltTheta[0] * D2R);
            F = (Mat_<double>(4, 4) << 1, 0.1, 0.000858375 * cosThetaX, 0, 0, 1, 0.0171675 * cosThetaX, 0.000858375 * cosThetaX, 0.000129285 * rollTheta * D2R * sinThetaT, 0, 1 - 0.0083534 * cosThetaX - 0.000129285 * cosThetaT, 0.1, 0.0025857 * rollTheta * D2R * sinThetaT, 0.000129285 * rollTheta * D2R * sinThetaT, -0.0025857 * cosThetaT - 0.1670687 * cosThetaX, 1 - 0.0083534 * cosThetaX - 0.000129285 * cosThetaT);
        }
        int rollControl(float desireTheta) {
            //##############################################
            //PID Roll Control
            //arg   : desireTheta {desire IDU ROLL theta [DEG]}
            //##############################################
            desireTheta = desireTheta * D2R;//Change RAD
            if (desireTheta != preDesireTheta) {
                PIDtermClear();
            }
            gain[2] = gain[1];
            gain[1] = gain[0];
            desireTheta = desireTheta * D2R;//Change RAD
            if (desireTheta != preDesireTheta) {
                PIDtermClear();
            }
            gain[2] = gain[1];
            gain[1] = gain[0];
            preDesireTheta = desireTheta;
            preDesireTheta = desireTheta;
            errTiltAngle[1] = errTiltAngle[0];
            errTiltAngle[0] = desireTheta - X.at<double>(1);
            pTerm = kp * errTiltAngle[0];
            iTerm += ki * errTiltAngle[0] * T;
            dTerm = kd * (errTiltAngle[0] - errTiltAngle[1]) / T;
            gain[0] = pTerm + iTerm - dTerm;
            Idu2tilt();
            return motor();
        }
        void Idu2tilt() {
            //IDU 2 tilt
        }
        int motor() {
            if (tiltInput[0] > 2.0944) {
                tiltInput[0] = 2.0944;
            }
            else if (tiltInput[0] < -2.0944) {
                tiltInput[0] = -2.0944;
            }
            else {
                tiltInput[0] = tiltInput[0] * R2D;
            }
            return floor(1500 + tiltInput[0] * R2D * 6.66667 + 0.5);
        }
        void PIDtermClear() {
            pTerm = 0;
            iTerm = 0;
            dTerm = 0;
        }
    };

    class IDU {
    private:
        float iduTheta[4] = { 0.0,0.0,0.0,0.0 }; //array resized is reqired
        //IDU STABLE PID GAIN
        float kp = 50;
        float ki = 00;
        float kd = 5;
        float pTerm = 0;
        float iTerm = 0;
        float dTerm = 0;
        double shellTheta[2] = { 0,0 };
        float err[2] = { 0.0,0.0 };
        float gain = 0;
    public:
        IDU(){}
        int stableControl(float pitch) {
            err[1] = err[0];
            err[0] = -pitch;
            pTerm = kp * err[0];
            iTerm += ki * err[0] * T;
            dTerm = kd * (err[0] - err[1]) / T;
            gain = pTerm + iTerm - dTerm+1500;
            
            if (gain>=2300){
                gain=2300;
            }
            else if (gain<=700){
                gain=700;
            }
            return floor(gain+ 0.5);
        }

        void PIDtermClear() {
            pTerm = 0;
            iTerm = 0;
            dTerm = 0;
        }
    };