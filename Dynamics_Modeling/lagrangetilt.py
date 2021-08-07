from sympy import diff, Function, symbols, cos, sin, latex, simplify
from sympy.physics.mechanics import *
# m_t, M_r, Jr, Jt, g, Rcm, rpin ,a = symbols("m_{t} M_{r} J_{r} J_{t} g R_{cm} r_{pin} a")
# variable define
m_t, M_r, Jr, Jt,Rcm= symbols("m_{t} M_{r} J_{r} J_{t}  R_{cm}")
a=6.382e-002
rpin=0.0175
g=9.81

th_t,  th_x = dynamicsymbols("\\theta_{t} \\theta_{x}")
U=m_t*g*(a*cos(th_x)+rpin*th_t*sin(th_x))-M_r*g*Rcm*cos(th_x)
T= 0.5*(Jr*th_x.diff()**2+Jt*th_x.diff()**2+m_t*rpin*rpin*th_t.diff()**2)
L = T - U
LM = LagrangesMethod(L, [th_t, th_x])
print(latex(simplify(LM.form_lagranges_equations())))

# %parameter
# Mr=2.64589817207665; % Robot Weight
# m_p=0.35875; % Pendulum Weight
# m_t= 0.38197027067833; % Tilt Weight with Driver Unit
# m_i=1.00447780288447; % NEED to Mesure
# m_o=0.51160374393088;% NEED to Mesure
# j_o=[7.99709410935264e-003, -1.41882636962701e-006, -1.53702376707172e-005,0;
#     -1.41882636962701e-006, 3.8940385229885e-003, -7.08020609180633e-005,0;
#     -1.53702376707172e-005,-7.08020609180633e-005, 7.22732484652406e-003, 0;
#     0 0 0 1];% NEED to Measure
# j_i=[4.14980277438829e-003, 3.69964214093724e-005, 9.58500096248825e-005 0;
#     3.69964214093724e-005, 4.42145001269003e-003, -8.52880895535963e-005 0;
#     9.58500096248825e-005, -8.52880895535963e-005, 1.13526880932078e-003 0;
#     0 0 0 1];% NEED to Measure
# j_pr=[1.22035638312589e-003, -4.89257920244134e-006, 2.8553665536466e-012 0;
#     -4.89257920244134e-006, 3.40181205693769e-004, 1.04387117424299e-010 0;
#     2.8553665536466e-012, 1.04387117424299e-010, 1.4874902226742e-003 0;
#     0 0 0 1];
# j_pl=[1.22035638312589e-003, -4.89257920244134e-006, 2.8553665536466e-012 0;
#     -4.89257920244134e-006, 3.40181205693769e-004, 1.04387117424299e-010 0;
#     2.8553665536466e-012, 1.04387117424299e-010, 1.4874902226742e-003 0;
#     0 0 0 1];
# j_t=[1.78703861879045e-003, 4.33482506384714e-006, 2.19644129372124e-006 0;
#     4.33482506384714e-006, 2.14789179188587e-004, -4.37292076784168e-004 0;
#     2.19644129372124e-006, -4.37292076784168e-004, 1.61327791148501e-003 0;
#     0 0 0 1];
# rpin=0.0175;
# rcm=[0.00040319,0.012421,-0.0015810]';
# lp=[-4.17096046467253e-004; -5.37035640317826e-002; -0.101055004237997; 1];% pendulum Center of Mass vector
# g=[0;0;-9.81;0];

# %Frame (Fixed ZYX) 
# T_01=[1 0 0 -x_o(t);0 1 0 0;0 0 1 0; 0 0 0 1]; % X-Axis Translate
# T_12=[1 0 0 0; 0 1 0 y_o(t);0 0 1 0;0 0 0 1]; % Y-Axis Translate
# R_23=[cos(th_oz(t)) sin(th_oz(t)) 0 0; -sin(th_oz(t)) cos(th_oz(t)) 0 0; 0 0 1 0; 0 0 0 1];% rotate Yaw
# R_34=[cos(th_oy(t)) 0 -sin(th_oy(t)) 0 ; 0 1 0 0;sin(th_oy(t)) 0 cos(th_oy(t)) 0 ; 0 0 0 1];% rotate pitch
# R_45=[1 0 0 0; 0 cos(th_ox(t)) sin(th_ox(t)) 0;0 -sin(th_ox(t)) cos(th_ox(t)) 0 ; 0 0 0 1];% roatate Roll
# R_56=[cos(th_iy(t)) 0 -sin(th_iy(t)) 0 ; 0 1 0 0;sin(th_iy(t)) 0 cos(th_iy(t)) 0 ; 0 0 0 1];% rotate pitch
# T_6pl=[cos(th_pl(t)) 0 -sin(th_pl(t)) 0;0,1,0 0.05370 ; sin(th_pl(t)) 0 cos(th_pl(t)) -0.44008 ; 0 0 0 1];
# T_6pr=[cos(th_pr(t)) 0 sin(th_pr(t)) 0;0,1,0 -0.05370 ; -sin(th_pr(t)) 0 cos(th_pr(t)) -0.44008 ; 0 0 0 1];

# % Shell


# % IDU
# r_idu=[5.80123011415123e-004; 9.16619278770412e-003; -1.92718560786594e-003; 1]; % NEED to Measure

j_s=7.99709410935264e-003
j_i=4.14980277438829e-003+1.00447780288447*((9.16619278770412*10**-3)**2+(-1.92718560786594*10**-3)**2)
j_pr=1.22035638312589e-003+0.35875*((-5.37035640317826e-002)**2+(-0.101055004237997)**2)
jr=j_s+j_i+j_pr*2
print(j_s)
print(j_i)
print(j_pr)

print(jr)