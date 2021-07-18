clc; clear;
syms x1 x2 x3 x4 u

a = 0.20459; b = 6.8e-4; c = 0.7056;
T = 0.1;
% x1 = th_s, x3 = th_p
f1 = x2;
f2 = -1/a*u;
f3 = x4;
f4 = -c/b*sin(x3)+1/b*u;
f = [f1;
    f2;
    f3;
    f4];

A = jacobian(f,[x1,x2,x3,x4]);
B = jacobian(f,[u]);
C = [0 1 0 0];
D = [0];

% F = expm(A)
F = eye(size(A)) + A*T + A^2*T^2/2 + A^3*T^3/6
% G = inv(A)*(F-eye(size(A)))*B
G = B*T + A*B*T^2/2 + A^2*B*T^3/6
H = C