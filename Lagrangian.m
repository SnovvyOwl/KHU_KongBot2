% This code is written by SnovvyOwl
% KHU-Kong-Bot 2 Project
% Calculate Largrangian
clear;clc;
syms x_o(t) y_o(t) th_oz(t) th_oy(t) th_ox(t) th_pr(t) th_pl(t) t th_t(t) th_iy(t) t
assume(x_o(t),'real')
assume(y_o(t),'real')
assume(th_oz(t),'real')
assume(th_oy(t),'real')
assume(th_ox(t),'real')
assume(th_iy(t),'real')
assume(th_t(t),'real')
assume(th_pr(t),'real')
assume(th_pl(t),'real')
assume(t,'positive')

%parameter
Mr=2.4614; % Robot Weight
m_p=0.35875; % Pendulum Weight
m_t= 0.381970; % Tilt Weight with Driver Unit
m_i=1; %NO VALUE
m_o=1;%NO VALUE
j_o=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%NO VALUE
j_i=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%NO VALUE
j_pr=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%NO VALUE
j_pl=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%NO VALUE
j_t=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%NO VALUE
rpin=0.005;% No Value
rcm=[0.00040319,0.012421,-0.0015810]';
lp=[0.00032; 0; -0.057047; 1];% pendulum Center of Mass vector
g=[0;0;9.81;0];

%Frame (Fixed ZYX) 
T_01=[1 0 0 -x_o(t);0 1 0 0;0 0 1 0; 0 0 0 1]; % X-Axis Translate
T_12=[1 0 0 0; 0 1 0 y_o(t);0 0 1 0;0 0 0 1]; % Y-Axis Translate
R_23=[cos(th_oz(t)) sin(th_oz(t)) 0 0; -sin(th_oz(t)) cos(th_oz(t)) 0 0; 0 0 1 0; 0 0 0 1];% rotate Yaw
R_34=[cos(th_iy(t)) 0 -sin(th_iy(t)) 0 ; 0 1 0 0;sin(th_iy(t)) 0 cos(th_iy(t)) 0 ; 0 0 0 1];% rotate pitch
R_45=[1 0 0 0; 0 cos(th_ox(t)) sin(th_ox(t)) 0;0 -sin(th_ox(t)) cos(th_ox(t)) 0 ; 0 0 0 1];% roatate Roll
th_5pml=[1 0 0 0;0 1 0 0.05370;0 0 1  -0.44008;0 0 0 1];
th_5pmr=[1 0 0 0;0 1 0 -0.05370;0 0 1  -0.44008;0 0 0 1];
R_pml=[cos(th_pl(t)) 0 -sin(th_pl(t)) 0;0,1,0 0 ; sin(th_pl(t)) 0 cos(th_pl(t)) 0 ; 0 0 0 1];
R_pmr=[cos(th_pr(t)) 0 sin(th_pr(t)) 0;0,1,0 0 ; -sin(th_pr(t)) 0 cos(th_pr(t)) 0 ; 0 0 0 1];

% Shell
V_oo=[0.3*diff(th_oy,t);0.3*diff(th_oy,t); 0;1];
w_5o=[diff(th_ox,t);diff(th_oy,t);diff(th_oz,t); 1]; 
w_oo=T_01*T_12*R_23*R_34*R_45*w_5o;
% IDU
r_idu=[0; 0; -1; 1]; % I don't have value.....
x_i=T_01*T_12*R_23*R_34*R_45*r_idu;
V_oi=diff(x_i,t);
th_oi=[th_ox(t);th_iy(t);th_oz(t); 1];
w_5i=diff(th_oi,t);
w_oi=T_01*T_12*R_23*R_34*R_45*w_5i;

% Pendulum R
x_pr=T_01*T_12*R_23*R_34*R_45*th_5pmr*R_pmr*lp;
V_pr=diff(x_pr,t);
w_pr=diff(T_01*T_12*R_23*R_34*R_45*[0; th_pr(t); 0; 1],t);
w_5pr=diff([0; th_pr(t); 0; 1],t);

% Pendulum L
x_pl=T_01*T_12*R_23*R_34*R_45*th_5pml*R_pml*lp;
V_pl=diff(x_pl,t);
w_pl=diff(T_01*T_12*R_23*R_34*R_45*[0; th_pl(t); 0; 1],t);
w_5pl=diff([0; th_pl(t); 0; 1],t);

% Tilt
rt=[1,-rpin*th_t(t),1 1]'; % I NEED VALUE
w_ot=w_oi;
w_5t=w_5i;
V_ot=diff(T_01*T_12*R_23*R_34*R_45*rt,t);

T=0.5*(m_o*(V_oo'*V_oo-1)+m_i*(V_oi'*V_oi-1)+m_p*(V_pr'*V_pr-1)+m_p*(V_pl'*V_pl-1)+(w_5o'*j_o*w_5o-1)+(w_5i'*j_i*w_5i-1)+(w_5pr'*j_pr*w_5pr-1)+(w_5pl'*j_pl*w_5pl-1)+(w_5t'*j_t*w_5t-1));
U=m_i*g'*x_i+m_p*g'*x_pr+m_p*g'*x_pl+m_t*g'*rt;
q=[th_iy(t)  th_pr(t) th_pl(t)  th_t(t) ];
dq=[diff(th_iy(t),t) diff(th_pr(t),t) diff(th_pl(t),t) diff(th_t(t),t)];

L=T-U;
EulerLagrange  = @(fun,t,q,dq) diff(diffDepVar(fun,dq),t) - diffDepVar(fun,q);
for i=1:length(q)
    dL(i)=EulerLagrange(L,t,q(i),dq(i));
end
dL=simplify(dL)'

function res = diffDepVar(fun,depVar)
syms xx
res = diff(subs(fun,depVar,xx),xx);
res = subs(res,xx,depVar);
end