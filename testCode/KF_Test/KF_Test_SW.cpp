#include<Matrix.h>
#include<iostream>
#include<vector>
#include<fstream>
#include<math.h>
#include<random>
#include<opencv2/opencv.hpp>
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
#define SIGMA_W 100
#define SIGMA_V 10
using namespace std;
using namespace cv;
double KF_filter(Matrix<double> &F,Matrix<double> &G,Matrix<double> &H,Matrix<double> &X, Matrix<double> &x_hat,Matrix<double>& GAMMA);
int main(){
    std::ofstream fout;
    fout.open("data.txt");
    //Matrix Define
    Mat F=(Mat_<double>(4,4)<<1,0.1,0,0,0,1,0,0,0,0,1,0.1,0,0, -0.00009637185, 0, -24.65561834,0);
    Mat G=(Mat_<double>(4,1)<<-0.0244, -0.4888,7.3529 ,147.0588);
    Mat H=(Mat_<double>(1,4)<<0,1,0,0);
    float D=0;
    float T= 0.1;
    //Initial Conditions
    Mat X_true=(Mat_<double>(4,1)<<0,0,1*DEG2RAD,0);
    std::cout<<X_true;
    Mat Gamma=(Mat_<double>(4,1)<<pow(T,4)/24,pow(T,3)/6,pow(T,2)/2,T); //SYSYTEM NOISE
    
    /*
    vector<vector<double>>gamma{{pow(T,4)/24},{pow(T,3)/6},{pow(T,2)/2},{T}};
    Matrix<double>GAMMA(4,1,gamma);
  
    double Q = 2* SIGMA_W*SIGMA_V*GAMMA.columns(); // 2를 곱하는 이유는 뭐지??
    double R= SIGMA_V*SIGMA_V;
*/
    //Gamma*randn(size(Gamma,2),1)
    /*
   


%% Initial Conditions
xp(:,1) = [0;0;0;0]; % guess of initial posteriori estimation
nx = length(xp(:,1)); % number of state
Pp = 1e3*eye(nx); % guess of initial error covariance
sigma_Pp(:,1) = sqrt(diag(Pp));
u(:,1)=[0]';
%% Noise
sigma_w = 100;        % system noise (std of acceleration)
sigma_v = 10;       % measurement noise (std of position sensor)
Q = 2*sigma_w^2*eye(size(Gamma,2));      % system noise covariance matrix
R = sigma_v^2;     % measurement noise covariance matrix

y=[];
%% KF Routine
t = 0:T:100;
for i = 1:length(t)-1
    %% True dynamics    
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
    
end



*/
    return 0;
}
double KF_filter(Matrix<double> &F,Matrix<double> &G,Matrix<double> &H,Matrix<double> &X, Matrix<double> &x_hat,Matrix<double>& GAMMA){
    default_random_engine generator;
    normal_distribution<double> distribution(5.0,2.0);

    double filtered_state=0;
    return filtered_state;
}