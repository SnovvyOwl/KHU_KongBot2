% This code is written by SnovvyOwl
% KHU-Kong-Bot 2 Project
% Calculate Largrangian
clear;clc;
syms x_o(t) y_o(t) t_oz(t) t_oy(t) t_ox(t) t_pr(t) t_pl(t) t t_t(t) t_iy(t)%t_enc(t)
assume(x_o(t),'real')
assume(y_o(t),'real')
%assume(t_enc(t),'real')

assume(t_oz(t),'real')
assume(t_oy(t),'real')
assume(t_ox(t),'real')
assume(t_iy(t),'real')

assume(t_t(t),'real')
assume(t_pr(t),'real')
assume(t_pl(t),'real')
Mr=2.4614; % Robot Weight
mp=0.35875; % Pendulum Weight
mt= 0.381970; % Tilt Weight with Driver Unit
rpin=0.005;
rcm=[0.00040319,0.012421,-0.0015810]';
lp=[0.00032, 0, -0.057047 1]';% pendulum Center of Mass vector
%Frame (Fixed ZYX) 
T_01=[1 0 0 -x_o(t);0 1 0 0;0 0 1 0; 0 0 0 1]; % X-Axis Translate
T_12=[1 0 0 0; 0 1 0 y_o(t);0 0 1 0;0 0 0 1]; % Y-Axis Translate
R_23=[cos(t_oz(t)) sin(t_oz(t)) 0 0; -sin(t_oz(t)) cos(t_oz(t)) 0 0; 0 0 1 0; 0 0 0 1];% rotate Yaw
R_34=[cos(t_oy(t)) 0 -sin(t_oy(t)) 0 ; 0 1 0 0;sin(t_oy(t)) 0 cos(t_oy(t)) 0 ; 0 0 0 1];% rotate pitch
R_45=[1 0 0 0; 0 cos(t_ox(t)) sin(t_ox(t)) 0;0 -sin(t_ox(t)) cos(t_ox(t)) 0 ; 0 0 0 1];% roatate Roll
T_5pml=[1 0 0 0;0 1 0 0.05370;0 0 1  -0.44008;0 0 0 1];
T_5pmr=[1 0 0 0;0 1 0 -0.05370;0 0 1  -0.44008;0 0 0 1];
R_pml=[cos(t_pl(t)) 0 sin(t_pl(t)) 0;0,1,0 0 ; -sin(t_pl(t)) 0 cos(t_pl(t)) 0 ; 0 0 0 1];
R_pmr=[cos(t_pr(t)) 0 sin(t_pr(t)) 0;0,1,0 0 ; -sin(t_pr(t)) 0 cos(t_pr(t)) 0 ; 0 0 0 1];
% Shell
V_o=[diff(x_o),diff(y_o) 0,1]';
w_oo=[diff(t_ox),diff(t_oy),diff(t_oz) 1]'; % t_ox and t_oz will get AHRS, t_oy will get Encoder...T_01*T_12*R_23*R_34*R_45*diff([0 t_enc(t) 0 0]')

% IDU
r_idu=[0 0 1 1]'; % I don't have value.....
v_oi=diff(T_01*T_12*R_23*R_34*R_45*r_idu);
t_oi=[t_ox(t),t_iy(t),t_oz(t) 1]';
w_oi=diff(t_oi);

% Pendulum R
v_pr=diff(T_01*T_12*R_23*R_34*R_45*T_5pmr*R_pmr*lp);
w_pr=diff(T_01*T_12*R_23*R_34*R_45*[0 t_pr(t) 0 1]');

% Pendulum L
v_pl=diff(T_01*T_12*R_23*R_34*R_45*T_5pml*R_pml*lp);
w_pl=diff(T_01*T_12*R_23*R_34*R_45*[0 t_pl(t) 0 1]');

% Tilt
rt=[1,-rpin*t_t(t),1 1]'
w_t=w_oi;
v_t=diff(T_01*T_12*R_23*R_34*R_45*rt)
