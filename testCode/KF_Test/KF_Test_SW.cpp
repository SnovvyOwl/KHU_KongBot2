#include<iostream>
#include<fstream>
#include<math.h>
#include<random>
#include<opencv2/opencv.hpp>
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
#define SIGMA_V 10 //Measurement Noise
#define T 0.1
using namespace std;
using namespace cv;
void KF_filter(Mat &x_bar, Mat &x_hat, Mat &X, Mat &U, Mat &z_hat, Mat &Z,Mat &p_bar,Mat &p_hat,Mat &S,Matx44d Q, double R, Matx44d &Qf,Mat &K);//KF
//GLOBAL VARIABLE
//NORMAL DISTRIBUTION RANDOM NUMBER GENERATOR
default_random_engine generator;
normal_distribution<double> dist_W(0,1.0);
normal_distribution<double> dist_V(0,1.0);
double W=0;//SYSTEM NOISE
double V=0;//SENSOR NOISE
Matx44d F(1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
Matx41d G(-0.0244, -0.4888,7.3529 ,147.0588);
Matx14d H(0,1,0,0);
int main(){
    ofstream fout;
    fout.open("data.txt");
    float D=0;
    //Initial Conditions
    //Matx41d X(0,0,0,0);
    Mat X=(Mat_<double>(4,1)<<0,0,0,0);
    Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
    Mat P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    sqrt(P,P);
    double W=dist_W(generator);
    Mat X_bar;
    X_bar.copySize(P.mul(W).diag());
    X_bar=X+P.mul(W).diag();
    P = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    Mat p_bar = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    Mat p_hat = Mat::eye(X_hat.rows, X_hat.rows, CV_64FC1)*1000;
    //Noise
    Matx44d Q(pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T,pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); //SYSYTEM NOISE 바꿀 필요가 있어보임
    double R= SIGMA_V*SIGMA_V;
    Mat u=(Mat_<double>(1,1)<<0);
    Matx44d Qf=Q;
    Mat z_hat;
    Mat Z;
    Mat K;
    Mat S;
    
    
    for(float i=0;i<100;i=i+T){
        fout<<X.t()<<endl;
        KF_filter(X_bar,X_hat,X,u,z_hat,Z,p_bar,p_hat,S,Q,R,Qf,K);
    }
    
    return 0;
}
void KF_filter(Mat &x_bar, Mat &x_hat, Mat &X, Mat &U, Mat &z_hat, Mat &Z,Mat &p_bar,Mat &p_hat,Mat &S,Matx44d Q, double R, Matx44d &Qf,Mat &K){
    double V= dist_V(generator);
    Z=H*X+sqrt(R)*V; //MESUREMENT MODEL
    //==================
    //MESUREMENT UPDATE
    //=================
    z_hat=H*x_bar;
    S=H*p_bar*H.t()+R;
    p_hat=p_bar-p_bar*H.t()*S.inv()*H*p_bar;
    K=p_bar*H.t()*S.inv();
    x_hat=x_bar+K*(Z-z_hat);
    //==================
    // TIME UPDATE
    // =================
    x_bar=F*x_hat+G*U;
    p_bar=F*p_hat*F.t()+Qf;
    //==================
    //System Dynamics
    //==================
    sqrt(Q,Q);
    double W=dist_W(generator);
    X=F*X+G*U+Q.diag()*W;
}