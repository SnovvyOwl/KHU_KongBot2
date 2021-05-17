#include<iostream>
#include<fstream>
#include<math.h>
#include<random>
#include<opencv2/opencv.hpp>
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
#define SIGMA_W 100 //System Noise
#define SIGMA_V 10 //Measurement Noise 
using namespace std;
using namespace cv;
double KF_filter(Matx44d &F,Matx41d&G,Matx14d &H,Mat &X_true, Mat &X_hat,Mat &U,Matx41d &G_w,double &W, double &V);//KF
//NORMAL DISTRIBUTION RANDOM NUMBER GENERATOR
default_random_engine generator;
normal_distribution<double> dist_W(0,1.0);
normal_distribution<double> dist_V(0,1.0);
double W=0;//SYSTEM NOISE
double V=0;//SENSOR NOISE
int main(){
    ofstream fout;
    fout.open("data.txt");
    //Matrix Define
    Matx44d F(1.0, 0.1, 0.0, 0.0, 0.0 ,1.0,0.0,0.0,0.0,0.0 ,1.0,0.1,0.0,0.0, -0.00009637185,1);
    Matx41d G(-0.0244, -0.4888,7.3529 ,147.0588);
    Matx14d H(0,1,0,0);
    float D=0;
    float T= 0.1;
    //Initial Conditions
    Mat X_true=(Mat_<double>(4,1)<<0,0,1*DEG2RAD,0);
    Mat X_hat=(Mat_<double>(4,1)<<0,0,0,0);
    //Noise
    Matx41d G_w(pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); //SYSYTEM NOISE 이걸 근데 이렇게 쓰는이유가 뭐지?
    double Q = 2* SIGMA_W*SIGMA_W*G_w.cols; // 2를 곱하는 이유는 뭐지??
    double R= SIGMA_V*SIGMA_V;
    Mat P = Mat::eye(X_hat.rows, X_hat.rows, CV_32F)*1000;//WHAT IS P?
    Mat sigma_P=P.diag();
    sqrt(sigma_P,sigma_P);
    Mat u=(Mat_<double>(1,1)<<0);
    Mat Z;
    
    W=dist_W(generator);
    KF_filter(F,G,H,X_true,X_hat,u,G_w,W,V);
    
/*

%% KF Routine
t = 0:T:100;
for i = 1:length(t)-1
*/
    return 0;
}
double KF_filter(Matx44d &F,Matx41d&G,Matx14d &H,Mat &X_true, Mat &X_hat,Mat &U,Matx41d &G_w, double &W, double &V){
    
 

   
    X_true=F*X_true+G*U+G_w*W;
    cout<<X_true;
    /*
    //True dynamics
    
    %x(:,i+1) = F*x(:,i)+G*u(:,i)+Gamma*randn(size(Gamma,2),1)*sigma_w; % system dynamics
    x(:,i+1) = F*x(:,i)+Gamma*randn(size(Gamma,2),1)*sigma_w;
    %% ====================================================
    %% Time update
    %% ====================================================
    %% Equation 1: Prediction of state
    %x_ = F*xp(:,i)+G*u(:,i);    
    x_ = F*xp(:,i);   
    %% Equation 2: Prediction of covariance
    P_ = F*Pp*F' + Gamma*Q*Gamma'; 
     %% compute Jacobian
    H =C;
    %% Measurement update
    %% ====================================================
    %% measurement generation
    y(:,i) = H*x(:,i+1) + diag(randn(size(sigma_v,1),1))*sigma_v;   
    %% Equation 3: Innovation Covariance
    S = H*P_*H'+R; 
    %% Equation 4: Residual
    nu = y(:,i)-H*x_;
    %% Equation 5: Kalman gain
    K = P_*H'*S^-1;   
    %% Equation 6: State update
    xp(:,i+1) = x_ + K * nu;
    %% Equation 7: Covariance update
    Pp = (eye(size(K*H))-K*H)*P_;
    %% =====================================================
    %% storing error covariance for ploting
    sigma_Pp(:,i+1) = sqrt(diag(Pp)); 
    %% storing Kalman gain for ploting
    K_store(i) = norm(K);

    default_random_engine generator;
    normal_distribution<double> distribution(5.0,2.0);
*/
    double filtered_state=0;
    return filtered_state;
}