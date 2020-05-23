syms tpml tpmr ttm tx;
% tpml =Theta_pml, tpmr = Theta_pmr ttm=Theta_tm tx= theta_x
Mr=2.4614; % Robot Weight
mp=0.35875; % Pendulum Weight
mt= 0.381970; % Tilt Weight with Driver Unit
rpml=[0 0.05370 -0.44008]; % left Pendulum Motor vector
rpmr=[0 -0.05370 -0.44008]; % right Pendulum Motor vector
rpin=0.005;
rcm=[0.00040319,0.012421,-0.0015810];
lp=[0.00032, 0, -0.057047];% pendulum Center of Mass vector 
rt=([1,0 ,0 ;0 cos(tx) -sin(tx); 0 -sin(tx) cos(tx)]*[-0.0032098,rpin*ttm,0.063823 ]')'; % Tilt Motor Vector
rpl=rpml+([cos(tpml) 0 sin(tpml);0,1,0; -sin(tpml) 0 cos(tpml)]*lp')' % left pendulum center of mass by tpml
rpr=rpmr+([cos(tpmr) 0 sin(tpmr);0,1,0; -sin(tpmr) 0 cos(tpmr)]*lp')' % right pendulum center of mass by tpmr
r_R=((Mr-2*mp-mt)*rcm+mp*rpl+mp*rpr+mt*rt)/Mr
rpl=simplify(rpl);
rpr=simplify(rpr);
r_R=simplify(r_R);
pretty(r_R)