%% Go Straight KHUKONBOT2
clc;
clear;
A= [0 1 0 0
    0 0 0 0
    0 0 0 1
    0 0 -9.6372*10^-4 0];
B=[0; -4.888;0 ;1470.59];
C=[0 1 0 0];
D=0;
d2r = pi/180; r2d = 1/d2r;

%% IDU exist Dynamic FOMULA
% A = [   0.            1.            0.            0.            0. 0.        ;
%  24.65561834    0.27506834   24.65561834    0.           24.65561834 0.  ;        
%   0.            0.            0.            1.            0. 0.;        
%  -24.65561834   -0.27506834  -24.65561834    0.          -24.65561834 0.;        
%  0.            0.            0.            0.            0. 1.        ;
%  -123.42648199   -0.37699722 -123.42648199    0.         -123.42648199 0.];
% 
% B = [  0.            0.        
%  -50.93858216   -9.43785913
%   0.            0.        
%   277.18292605 -443.05082865
%     0.            0.        
%  -156.43004347  592.32737796];
% 
% C=[0 1 0 0 0 0;
%     1 0 1 0 0 0];
% D=[0 0;0 0];
%% SET F, G
T=0.1;
Gamma = [T^4/24; T^3/6; T^2/2; T]; % Gamma (related to system noise)
F = expm(A*T)

% ANALTIC SOLUTION
%G=inv(A)*(F-eye(4))*B

% NUMERICAL SOLUTION of G 
G= B*T+A*B*T^2/2+A^2*B*T^3/6

sys=ss(A,B,C,D);
sys_discrete=c2d(sys,T)
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

figure; clf; hold on;
plot(t, x(2,:),'-b');                  % plot the real plant behavior
plot(t,xp(2,:),'.-r');                 % plot the Kalman filter prediction over the plant;
legend('real','KF')