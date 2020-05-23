syms tpml tpmr ttm;
% tpml =Theta_pml, tpmr = Theta_pmr ttm=Theta_tm
Mr=2.4614; % Robot Weight
mp=0.35875; % Pendulum Weight
mt= 0; % Tilt Weight with Driver Unit
rpml=[-0.0001 0.05370 -0.44008]; % left Pendulum Motor vector
rpmr=[-0.0001 -0.05370 -0.44008]; % right Pendulum Motor vector
lp=[-0.003171, -0.00000791, -0.05704743];% pendulum Center of Mass vector 
rt=[]; % Tilt Motor Vector
rpl=rpml+([cos(tpml) 0 sin(tpml);0,1,0; -sin(tpml) 0 cos(tpml)]*lp')'; % left pendulum center of mass by tpml
rpr=rpmr+([cos(tpmr) 0 sin(tpmr);0,1,0; -sin(tpmr) 0 cos(tpmr)]*lp')'; % right pendulum center of mass by tpml
% r_R=((Mr-2*mp-mt)*rcm+mp*rpl+mp*rpr+mt*rt)/Mr;