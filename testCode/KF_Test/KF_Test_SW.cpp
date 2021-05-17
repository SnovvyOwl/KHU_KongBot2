#include<iostream>
#include<fstream>
#include<math.h>
#include<random>
#include<opencv2/opencv.hpp>
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
#define SIGMA_W 100 //System Noise
#define SIGMA_V 10 //Measurement Noise
#define T 0.1
using namespace std;
using namespace cv;
void KF_filter(Matx44d &F,Matx41d&G, Matx14d &H, Mat &X_bar, Mat &X_hat, Mat &x, Mat &U, Mat &Z,Matx41d &G_w, double &W, double &R);//KF
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
    //Matrix Defin

    float D=0;
    //Initial Conditions
    
    Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
    Mat P = Mat::eye(X_hat.rows, X_hat.rows, CV_32F)*1000;//WHAT IS P?
    cout<<P;
    Mat X_bar=(Mat_<double>(4,1)<<0,0,1*DEG2RAD,0);
    sqrt(P,P);
    X_bar=X_bar+P;
    cout<<P;
    //Noise
    Matx41d G_w(pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); //SYSYTEM NOISE 이걸 근데 이렇게 쓰는이유가 뭐지?
    double Q = 2* SIGMA_W*SIGMA_W*G_w.cols; // 2를 곱하는 이유는 뭐지??
    double R= SIGMA_V*SIGMA_V;
    Mat sigma_P=P.diag();
    sqrt(sigma_P,sigma_P);
    Mat u=(Mat_<double>(1,1)<<0);
    Mat Z;
    //KF_filter(F,G,H,X_true,X_hat,u,Z,G_w,W,V);
    /*
    for(float i=0;i<100;i=i+T){
        fout<<X_true<<endl;
        KF_filter(F,G,H,X_true,X_hat,u,G_w,W,V);
    }
    */
    return 0;
}
void KF_filter(Mat &x_bar, Mat &x_hat, Mat &X, Mat &U, Mat &z_bar,Mat &z_hat, Mat &Z,Mat &p_bar,Mat &p_hat,Mat Q, Mat R, Mat &Qf){
    sqrt(R,R);
    Z=H*X+R*dist_V(generator); //MESUREMENT MODEL
    //==================
    //MESUREMENT UPDATE
    //==================
    z_hat=H*x_bar;
    Mat S=H*p_bar*H.t()+R;
    p_hat=p_bar-p_bar*H.t()*S.inv()*H*p_bar;
    Mat K=p_bar*H.t()*S.inv();
    x_hat=x_bar+K*(Z-z_hat);
    //==================
    // TIME UPDATE
    // =================
    x_bar=F*x_hat+G*U;
    p_bar=F*p_hat*F.t()+Qf;
    sqrt(Q,Q);
    X=F*X+G*U+Q*dist_W(generator);

}