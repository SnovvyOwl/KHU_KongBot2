from sympy import diff, Function, symbols, cos, sin, latex, simplify
from sympy.physics.mechanics import *
# m_t, M_r, Jr, Jt, g, Rcm, rpin ,a = symbols("m_{t} M_{r} J_{r} J_{t} g R_{cm} r_{pin} a")
# variable define
m_t, M_r, Jr, Jt,Rcm= symbols("m_{t} M_{r} J_{r} J_{t}  R_{cm}")
a=6.382e-002
rpin=0.0175
g=9.81
###############
#cal JR
ms=0.51
mi=1.00
mp=0.36*2
Rs=0.15
Js=3.89*10**-3
Ji=4.42*10**-3
mt=0.38197
Jp=3.4*10**-4*2
Jr=0.024072249572891516
Jt=1.78703861879045e-003
g=9.81 
lp=0.1
rpin=0.0175
ri=1.92718560786594e-003
a=6.382e-002
Mr=2.64589817207665
Rcm=(mi*ri+2*mp*lp+mt*a)/Mr
print(Rcm)
j_s=7.99709410935264e-003
j_i=4.14980277438829e-003+1.00447780288447*((9.16619278770412*10**-3)**2+(-1.92718560786594*10**-3)**2)
j_pr=1.22035638312589e-003+0.35875*((-5.37035640317826e-002)**2+(-0.101055004237997)**2)
jr=j_s+j_i+j_pr*2
print(j_s)
print(j_i)
print(j_pr)
print(jr)
######################################################
th_t,  th_x = dynamicsymbols("\\theta_{t} \\theta_{x}")
U=m_t*g*(a*cos(th_x)+rpin*th_t*sin(th_x))-M_r*g*Rcm*cos(th_x)
T= 0.5*(Jr*th_x.diff()**2+Jt*th_x.diff()**2+m_t*rpin*rpin*th_t.diff()**2)
L = T - U
LM = LagrangesMethod(L, [th_t, th_x])
print(latex(simplify(LM.form_lagranges_equations())))