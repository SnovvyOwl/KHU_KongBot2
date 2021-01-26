from sympy import diff, Function, symbols,cos, sin, latex, simplify
from sympy.physics.mechanics import *
# variable define
ms, mi, mp, Rs, Js, Ji, Jp, t, g ,lp= symbols("m_{s} m_{i} m_{p} R_{s} J_{s} J_{i} J_{p} t g l_{p}")
"""
    m_{s}= mass of Shell
    m_{i}= mass of IDU
    m_{p}= mass of pendulums
    R_{s}= Radius of Shell (Scala)
    J_{s}= inertia of Shell
    J_{i}= inertia of IDU
    J_{p}= inertia of pendulums (rotational axis)
    g = gravity accelation (scala)
    l_{p}= length of pendulums C.M to rotational Axis
    \\theta_{s}= angle of Shell (Global Frame)
    \\theta_{im}= angle of IDU motor (Shell Frame)
    \\theta_{pm}=angle of pendulum motors (IDU Frame)

    \\theta_{i}=\\theta_{im}+\\theta_{s}                        (Global Frame)
    \\theta_{p}=\\theta_{im}+\\theta_{s}+\\theta_{pm}           (Global Frame)
"""

th_s,th_im,th_pm=dynamicsymbols("\\theta_{s} \\theta_{im} \\theta_{pm}")

# Energy Equation
Ts=1/2*ms*(Rs*  th_s.diff())**2+1/2*Js* th_s.diff()**2
Ti=1/2*mi*(Rs*  th_s.diff())**2+1/2*Ji*( th_s.diff()+  th_im.diff())**2
Tp=1/2*mp*((Rs* th_s.diff()+lp*( th_s.diff()+ th_im.diff()+ th_pm.diff())*cos(th_s+th_im+th_pm))**2+lp*( th_s.diff()+ th_im.diff()+ th_pm.diff()*sin(th_s+th_im+th_pm))**2)+1/2*Jp*( th_s.diff()+ th_im.diff()+ th_pm.diff())**2
T=Ts+Ti+Tp
U=-mp*g*lp*cos(th_s+th_im+th_pm)
L=T-U
LM=LagrangesMethod(L,[th_s,th_im,th_pm])
print(latex(simplify(LM.form_lagranges_equations())))
