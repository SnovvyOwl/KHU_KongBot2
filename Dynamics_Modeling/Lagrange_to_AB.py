import numpy as np
from sympy import diff, Function, symbols,cos, sin, latex, simplify
import sympy as sym
from sympy.physics.mechanics import *
ms, mi, mp, Rs, Js, Ji, Jp, t, g ,lp, Ti,Tp= symbols("m_{s} m_{i} m_{p} R_{s} J_{s} J_{i} J_{p} t g l_{p},\\tau_{i},\\tau_{p}")
th_s,th_im,th_pm=dynamicsymbols("\\theta_{s} \\theta_{im} \\theta_{pm}")

z=sym.Matrix([[mp*lp*Rs, Ji+Jp+Js+mi*Rs**2+ms*Rs**2+mp*Rs*(Rs+lp),0,Ji+Jp,0,Jp],[ 0,Ji+Jp+mp*lp*Rs+mp*lp**2,0,Ji+Jp+mp*lp**2,0,Jp+mp*lp**2],[0,Jp+mp*lp*Rs+mp*lp**2,0,Jp+mp*lp**2,0,Jp+mp*lp**2]])
a=sym.Matrix([[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0]])
u=sym.Matrix([[-Ti],[Ti-Tp],[Tp]])

Z=z.T*z
print(Z.shape)
#B=Z.inv()*z.T
#A=B*a


#print(A)

#print(B)