clc
clear

A = [   0.            1.            0.            0.            0. 0.        ;
 24.65561834    0.27506834   24.65561834    0.           24.65561834 0.  ;        
  0.            0.            0.            1.            0. 0.;        
 -24.65561834   -0.27506834  -24.65561834    0.          -24.65561834 0.;        
 0.            0.            0.            0.            0. 1.        ;
 -123.42648199   -0.37699722 -123.42648199    0.         -123.42648199 0.];

B = [  0.            0.        
 -50.93858216   -9.43785913
  0.            0.        
  277.18292605 -443.05082865
    0.            0.        
 -156.43004347  592.32737796];

C=[0 1 0 0 0 0;
    1 0 1 0 0 0];
    

D=[0 0;0 0];
[num, den]=ss2tf(A,B,C,D,1);

num11=[0, -31.28, 0, -2804];
num12=[0, 0, 226.2, 0];
num13=[0, 0, 42.87, 0];
num21=[0, 29.93, 0, 1402];
num22=[0, 0, -452.5, 0];
num23=[0, 0, 85.88, -21.44];

den11=[1, -0.1689, 75.8, -15.14];
den12=[1, 0, 0, 0];
den13=den11;
den21=den11;
den22=den12;
den23=den11;

g=tf(num23,den23);
rlocus(g)