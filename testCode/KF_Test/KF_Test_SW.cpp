#include<Matrix.h>
#include<iostream>
#include<vector>
#include<fstream>
#include<math.h>
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI
using namespace std;
double KF_filter(Matrix<double>& m,double state, double ex_state);
int main(){
    ofstream fout;
    double state=0;
    double ex_state=0;
    fout.open("data.txt");
    //Matrix Define
    vector<vector<double>> f  
    {
        {1,0.1,0,0},
        {0,1,0,0},        
        {0,0,1,0.1},        
        {0,0, -0.00009637185, 0, -24.65561834,0}
    };
    vector<vector<double>>g{{-0.0244}, {-0.4888},{7.3529} ,{147.0588}};
    vector<vector<double>>h{{0, 1, 0, 0}}; 
    Matrix<double>F(4,4,f);
    Matrix<double>G(4,1,g);
    Matrix<double>H(1,4,h);
    double D=0; 
    
    //Initial Conditions
    float T= 0.1;
    vector<vector<double>>x{{0},{0},{1*DEG2RAD},{0}};
    Matrix<double>X(4, 1,x);
    /*
    Gamma = [T^4/24; T^3/6; T^2/2; T]; % Gamma (related to system noise)

%% Initial Conditions
x(:,1) = [0;0;1*d2r;0]; % true initial state
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
double KF_filter(Matrix<double> &m,double state, double ex_state){
    double filtered_state=0;
    return filtered_state;
}