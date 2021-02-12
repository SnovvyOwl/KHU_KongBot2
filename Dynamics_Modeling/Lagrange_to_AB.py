import numpy as np
import control
import matplotlib.pyplot

""""
ms, mi, mp, Rs, Js, Ji, Jp, t, g ,lp, Ti,Tp= symbols("m_{s} m_{i} m_{p} R_{s} J_{s} J_{i} J_{p} t g l_{p},\\tau_{i},\\tau_{p}")
th_s,th_im,th_pm=dynamicsymbols("\\theta_{s} \\theta_{im} \\theta_{pm}")

z=sym.Matrix([[-mp*lp*Rs, Ji+Jp+Js+mi*Rs**2+ms*Rs**2+mp*Rs*(Rs+lp),0,Ji+Jp,0,Jp],[ 0,Ji+Jp+mp*lp*Rs+mp*lp**2,0,Ji+Jp+mp*lp**2,0,Jp+mp*lp**2],[0,Jp+mp*lp*Rs+mp*lp**2,0,Jp+mp*lp**2,0,Jp+mp*lp**2]])
a=sym.Matrix([[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0]])
u=sym.Matrix([[-Ti],[Ti-Tp],[Tp]])
Z=z.T*z
print(Z.shape)
#B=Z.inv()*z.T
#A=B*a


#print(A)

#print(B)
"""


ms=0.51
mi=1.00
mp=0.36*2
Rs=0.15
Js=3.89*10**-3
Ji=4.42*10**-3
Jp=3.4*10**-4*2
g=9.81
lp=0.1

z=np.array([[1,0,0,0,0,0],[-mp*lp*Rs, Ji+Jp+Js+mi*Rs**2+ms*Rs**2+mp*Rs*(Rs+lp),0,Ji+Jp,0,Jp],[0,0,1,0,0,0],[ 0,Ji+Jp+mp*lp*Rs+mp*lp**2,0,Ji+Jp+mp*lp**2,0,Jp+mp*lp**2],[0,0,0,0,1,0],[0,Jp+mp*lp*Rs+mp*lp**2,0,Jp+mp*lp**2,0,Jp+mp*lp**2]])
k=np.array([[0,1,0,0,0,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[0,0,0,1,0,0],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0],[0,0,0,0,0,1],[-mp*lp*g,0,-mp*lp*g,0,-mp*lp*g,0]])
T=np.array([[0,0],[-1,0],[0,0],[1,-1],[0,0],[0,1]])

B=np.matmul(np.linalg.inv(z),T)
A=np.matmul(np.linalg.inv(z),k)
C=np.array([[0,1,0,0,0,0],[1,0,1,0,0,0],[1,0,1,0,1,0]])
D=0
tfsys=control.ss2tf(A,B,C,D)
sssys=control.ss(A,B,C,D)

print(tfsys)