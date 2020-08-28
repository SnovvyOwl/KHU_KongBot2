% This code is written by SnovvyOwl
% KHU-Kong-Bot 2 Project
% Calculate Largrangian
clear;clc;
syms x_o(t) y_o(t) th_oz(t) th_oy(t) th_ox(t) th_iy(t) th_pr(t) th_pl(t) th_t(t) t 
assume(x_o(t),'real')
assume(y_o(t),'real')
assume(th_oz(t),'real')
assume(th_oy(t),'real')
assume(th_ox(t),'real')
assume(th_iy(t),'real')
assume(th_t(t),'real')
assume(th_pr(t),'real')
assume(th_pl(t),'real')
assume(t,'real')

%parameter
Mr=2.64589817207665; % Robot Weight
m_p=0.35875; % Pendulum Weight
m_t= 0.38197027067833; % Tilt Weight with Driver Unit
m_i=1.00447780288447; % NEED to Mesure
m_o=0.51160374393088;% NEED to Mesure
j_o=[7.99709410935264e-003, -1.41882636962701e-006, -1.53702376707172e-005,0;
    -1.41882636962701e-006, 3.8940385229885e-003, -7.08020609180633e-005,0;
    -1.53702376707172e-005,-7.08020609180633e-005, 7.22732484652406e-003, 0;
    0 0 0 1];% NEED to Measure
j_i=[4.14980277438829e-003, 3.69964214093724e-005, 9.58500096248825e-005 0;
    3.69964214093724e-005, 4.42145001269003e-003, -8.52880895535963e-005 0;
    9.58500096248825e-005, -8.52880895535963e-005, 1.13526880932078e-003 0;
    0 0 0 1];% NEED to Measure
j_pr=[1.22035638312589e-003, -4.89257920244134e-006, 2.8553665536466e-012 0;
    -4.89257920244134e-006, 3.40181205693769e-004, 1.04387117424299e-010 0;
    2.8553665536466e-012, 1.04387117424299e-010, 1.4874902226742e-003 0;
    0 0 0 1];
j_pl=[1.22035638312589e-003, -4.89257920244134e-006, 2.8553665536466e-012 0;
    -4.89257920244134e-006, 3.40181205693769e-004, 1.04387117424299e-010 0;
    2.8553665536466e-012, 1.04387117424299e-010, 1.4874902226742e-003 0;
    0 0 0 1];
j_t=[1.78703861879045e-003, 4.33482506384714e-006, 2.19644129372124e-006 0;
    4.33482506384714e-006, 2.14789179188587e-004, -4.37292076784168e-004 0;
    2.19644129372124e-006, -4.37292076784168e-004, 1.61327791148501e-003 0;
    0 0 0 1];
rpin=0.0175;
rcm=[0.00040319,0.012421,-0.0015810]';
lp=[-4.17096046467253e-004; -5.37035640317826e-002; -0.101055004237997; 1];% pendulum Center of Mass vector
g=[0;0;-9.81;0];

%Frame (Fixed ZYX) 
T_01=[1 0 0 -x_o(t);0 1 0 0;0 0 1 0; 0 0 0 1]; % X-Axis Translate
T_12=[1 0 0 0; 0 1 0 y_o(t);0 0 1 0;0 0 0 1]; % Y-Axis Translate
R_23=[cos(th_oz(t)) sin(th_oz(t)) 0 0; -sin(th_oz(t)) cos(th_oz(t)) 0 0; 0 0 1 0; 0 0 0 1];% rotate Yaw
R_34=[cos(th_oy(t)) 0 -sin(th_oy(t)) 0 ; 0 1 0 0;sin(th_oy(t)) 0 cos(th_oy(t)) 0 ; 0 0 0 1];% rotate pitch
R_45=[1 0 0 0; 0 cos(th_ox(t)) sin(th_ox(t)) 0;0 -sin(th_ox(t)) cos(th_ox(t)) 0 ; 0 0 0 1];% roatate Roll
R_56=[cos(th_iy(t)) 0 -sin(th_iy(t)) 0 ; 0 1 0 0;sin(th_iy(t)) 0 cos(th_iy(t)) 0 ; 0 0 0 1];% rotate pitch
T_6pl=[cos(th_pl(t)) 0 -sin(th_pl(t)) 0;0,1,0 0.05370 ; sin(th_pl(t)) 0 cos(th_pl(t)) -0.44008 ; 0 0 0 1];
T_6pr=[cos(th_pr(t)) 0 sin(th_pr(t)) 0;0,1,0 -0.05370 ; -sin(th_pr(t)) 0 cos(th_pr(t)) -0.44008 ; 0 0 0 1];

% Shell
V_0o=[diff(x_o(t),t);diff(y_o(t),t); 0;1];
w_0o=[diff(th_ox,t);diff(th_oy,t);diff(th_oz,t); 1];

% IDU
r_idu=[5.80123011415123e-004; 9.16619278770412e-003; -1.92718560786594e-003; 1]; % NEED to Measure
x_i=T_01*T_12*R_23*R_34*R_45*R_56*r_idu;
V_0i=diff(x_i,t);
th_5i=[0;th_iy(t);0; 1];
w_5i=diff(th_5i,t);
w_0i=T_01*T_12*R_23*R_34*R_45*w_5i;


% Pendulum R
x_pr=T_01*T_12*R_23*R_34*R_45*R_56*T_6pr*lp;
V_0pr=diff(x_pr,t);
w_0pr=diff(T_01*T_12*R_23*R_34*R_45*R_56*[0; th_pr(t); 0; 1],t);
w_6pr=[0; th_pr(t); 0; 1];


% Pendulum L
x_pl=T_01*T_12*R_23*R_34*R_45*R_56*T_6pl*lp;
V_0pl=diff(x_pl,t);
w_0pl=diff(T_01*T_12*R_23*R_34*R_45*R_56*[0; th_pl(t); 0; 1],t);
w_6pl=[0; th_pl(t); 0; 1];



% Tilt
rt=[-3.2098005734442e-004, -1.79374587721968e-002-rpin*th_t(t),6.38233185099966e-002, 1]';% NEED to Measure
w_0t=w_0i;
V_0t=diff(T_01*T_12*R_23*R_34*R_45*R_56*rt,t);
w_6t=inv(R_56)*inv(R_45)*inv(R_34)*inv(R_23)*w_0t;

% Energy EQN
th_i=th_oy+th_iy; % motor ouput


T=0.5*(m_o*(V_0o'*V_0o-1)+m_i*(V_0i'*V_0i-1)+m_p*(V_0pr'*V_0pr-1)+m_p*(V_0pl'*V_0pl-1)+m_t*(V_0t'*V_0t-1)+(w_0o'*j_o*w_0o-1)+(w_5i'*j_i*w_5i-1)+(w_6pr'*j_pr*w_6pr-1)+(w_6pl'*j_pl*w_6pl-1)+(w_6t'*j_t*w_6t-1));
U=m_i*g'*x_i+m_p*g'*x_pr+m_p*g'*x_pl+m_t*g'*rt;
q=[th_pl(t),th_pr(t),th_i(t),th_t(t)];
dq=[diff(th_pl(t),t), diff(th_pr(t),t), diff(th_i(t),t),diff(th_t(t),t)];

assume(q,'real')

L=T+U;

% SOLVE Lagrange
EulerLagrange  = @(fun,t,q,dq) diff(diffDepVar(fun,dq),t) - diffDepVar(fun,q);
for i=1:length(q)
    dL(i)=EulerLagrange(L,t,q(i),dq(i));
end

%ANSWER
% dL=simplify(dL)'
assume(dL,'real');
dL'

function res = diffDepVar(fun,depVar)
syms xx
res = diff(subs(fun,depVar,xx),xx);
res = subs(res,xx,depVar);
end
