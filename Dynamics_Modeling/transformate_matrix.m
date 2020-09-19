clc;clear;
syms theta_oz theta_oy theta_ox theta_iy x_0 y_0 theta_t theta_pl theta_pr
T_05=[cos(theta_oz)*cos(theta_oy), cos(theta_oz)*sin(theta_oy)*sin(theta_ox)-sin(theta_oz)*cos(theta_ox),cos(theta_oz)*sin(theta_oy)*cos(theta_ox)+sin(theta_oz)*sin(theta_ox),-x_0
    sin(theta_oz)*cos(theta_oy),sin(theta_oz)*sin(theta_oy)*sin(theta_ox)+cos(theta_oz)*cos(theta_ox),sin(theta_oz)*sin(theta_oy)*cos(theta_ox)-cos(theta_oz)*sin(theta_ox),y_0
    -sin(theta_oy),cos(theta_oy)*sin(theta_ox),cos(theta_oy)*cos(theta_ox),0
    0,0,0,1]
T_56=[cos(theta_iy) 0 sin(theta_iy) 0; 0 1 0 0;-sin(theta_iy) 0 cos(theta_iy) 0;0 0 0 1 ];
T_06=T_05*T_56
T_6pl=[cos(theta_pl) 0 -sin(theta_pl) 0;0,1,0 0.05370 ; sin(theta_pl) 0 cos(theta_pl) -0.44008 ; 0 0 0 1];
T_6pr=[cos(theta_pr) 0 sin(theta_pr) 0;0,1,0 -0.05370 ; -sin(theta_pr) 0 cos(theta_pr) -0.44008 ; 0 0 0 1];
T_0pl=T_06*T_6pl
T_0pr=T_06*T_6pr
T_6t=[1,0,0, -3.2098005734442e-004;0,1,0, -1.79374587721968e-002-0.0175*theta_t; 0, 0, 1, 6.38233185099966e-002;0,0,0,1];
T_0t=T_06*T_6t
T_0t=simplifyFraction(T_0t)

