% Go Straight KHUKONBOT2
A= [0 1 0 0
    0 0 0 0
    0 0 1 0
    0 0 -9.6372*10^-4 0];
B=[0; -4.888;0 ;14070.59];
C=[0 1 0 0];
D=0;
T=0.1;
F = expm(A*T)
% ANALTIC SOLUTION
% G=inv(A)*(F-eye(4))*B

% NUMERICAL SOLUTION of G 
G = B*T+A*B*T^2/2+A^2*B*T^3/6

sys=ss(A,B,C,D);
sys_discrete=c2d(sys,T)