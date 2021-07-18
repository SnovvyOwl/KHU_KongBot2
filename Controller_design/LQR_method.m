clc;
clear;
A= [0 1 0 0
    0 0 0 0
    0 0 0 1
    0 0 -9.6372*10^-4 0];
B=[0; -4.888;0 ;1470.59];
C=[0 1 0 0];
D=0;
Q = [15000 0 0 0
     0 100000 0 0
     0 0 1000 0
     0 0 0 100]
 R = [1];

 [K,S,e]= lqr(A,B,Q,R)
% k1=K(1); k2=K(2);k3=K(3);k4=K(4)

% AA = A - B*K;
% BB = B*k1;
% CC=C;
% DD=D;
% t = 0:0.01:10;
% [y,x,t]=step(AA,BB,CC,DD,1,t);
% plot(t,x)
% legend('x1','x2','x3','x4')
% grid

d2r = pi/180; r2d = 1/d2r;

sys=ss(A-B*K,eye(4),eye(4),eye(4))
t=0:0.01:500;
x=initial(sys,[0;1;0;0],t);
X1=[1 0 0 0]*x';
X2=[0 1 0 0]*x';
X3=[0 0 1 0]*x';
X4=[0 0 0 1]*x';

 subplot(2,2,1);plot(t,X1);grid
 subplot(2,2,2);plot(t,X2);grid
 subplot(2,2,3);plot(t,X3);grid
 subplot(2,2,4);plot(t,X4);grid