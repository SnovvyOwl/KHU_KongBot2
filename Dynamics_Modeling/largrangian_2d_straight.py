from sympy import diff, Function, symbols,cos, sin, latex, simplify
from sympy.physics.mechanics import *
# variable define
ms, mi, mp, Rs, Js, Ji, Jp, t, g ,lp= symbols("m_{s} m_{i} m_{p} R_{s} J_{s} J_{i} J_{p} t g l_{p}")
"""
th_s=Function("\\theta_{s}")(t)
th_im= Function("\\theta_{im}")(t) 
th_pm=Function("\\theta_{pm}")(t)
"""

th_s,th_im,th_pm=dynamicsymbols("\\theta_{s} \\theta_{im} \\theta_{pm}")

# Energy Equation
Ts=1/2*ms*(Rs*  th_s.diff())**2+1/2*Js* th_s.diff()**2
Ti=1/2*mi*(Rs*  th_s.diff())**2+1/2*Ji*( th_s.diff()+  th_im.diff())**2
Tp=1/2*mp*((Rs* th_s.diff()+( th_s.diff()+ th_im.diff()+ th_pm.diff())*cos(th_s+th_im+th_pm))**2+( th_s.diff()+ th_im.diff()+ th_pm.diff()*sin(th_s+th_im+th_pm))**2)+1/2*Jp*( th_s.diff()+ th_im.diff()+ th_pm.diff())**2
T=Ts+Ti+Tp
U=-mp*g*lp*cos(th_s+th_im+th_pm)
L=T-U
LM=LagrangesMethod(L,[th_s,th_im,th_pm])
print(latex(simplify(LM.form_lagranges_equations())))