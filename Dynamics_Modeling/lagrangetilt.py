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