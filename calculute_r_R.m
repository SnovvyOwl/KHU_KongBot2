syms tpml tpmr ttm;
Mr=2.4614;
mp=0.35875;
mt= 0;
rpml=[-0.0001 0.05370 -0.44008];
rpmr=[-0.0001 -0.05370 -0.44008];
lp=[-0.003171, -0.00000791, -0.05704743];
rt=[];
rpl=rpml+([cos(tpml) 0 sin(tpml);0,1,0; -sin(tpml) 0 cos(tpml)]*lp')';
rpr=rpmr+([cos(tpmr) 0 sin(tpmr);0,1,0; -sin(tpmr) 0 cos(tpmr)]*lp')';
rpr=simplify(rpr)
% r_R=((Mr-2*mp-mt)*rcm+mp*rpl+mp*rpr+mt*rt)/Mr;