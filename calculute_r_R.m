syms tpml tpmr ttm tx ty tz;
assume(ttm,'real')
% tpml =Theta_pml, tpmr = Theta_pmr, ttm=Theta_tm, tx= theta_x ty=theta_y
Mr=2.4614; % Robot Weight
mp=0.35875; % Pendulum Weight
mt= 0.381970; % Tilt Weight with Driver Unit
rpml=[0 0.05370 -0.44008]'; % left Pendulum Motor vector
rpmr=[0 -0.05370 -0.44008]'; % right Pendulum Motor vector
rpin=0.005;
rcm=[0.00040319,0.012421,-0.0015810]';
lp=[0.00032, 0, -0.057047]';% pendulum Center of Mass vector 
R=[1, 0 ,0 ;0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)]; % Rotate Matrix tx  AXIS X
P=[cos(ty) 0 sin(ty);0,1,0; -sin(ty) 0 cos(ty)]; % Rotate Matrix ty  AXIS Y
Y=[cos(tz) -sin(tz) 0 ; sin(tz)  cos(tz) 0 ; 0 0 1]; % Rotate Matrix tz  AXIS Z
rt=[-0.0032098,rpin*ttm,0.063823 ]'; % Tilt Motor Vector
rpl=rpml+([cos(tpml) 0 sin(tpml);0,1,0; -sin(tpml) 0 cos(tpml)]*lp); % left pendulum center of mass by tpml
rpr=rpmr+([cos(tpmr) 0 sin(tpmr);0,1,0; -sin(tpmr) 0 cos(tpmr)]*lp); % right pendulum center of mass by tpmr
r_R=Y*P*R*(((Mr-2*mp-mt)*rcm+mp*rpl+mp*rpr+mt*rt)/Mr);
r_R=simplify(r_R);
Mrg=Mr*[0 ,0, -9.81]';
countermass=cross(r_R,Mrg);
vec_rR=ccode(r_R)
tau_c=ccode(countermass)
file1=fopen("r_R.txt",'w');
file2=fopen("tau_rc.txt",'w');
fprintf(file1,vec_rR);
fprintf(file2,tau_c);
fclose(file1);
fclose(file2);