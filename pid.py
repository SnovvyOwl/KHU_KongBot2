import sympy
import numpy as np
import control
Kp,Ki,Kd,z,T=sympy.symbols("Kp Ki Kd z T")
PID=Kp+Ki*T*z/(z-1)+Kd*(z-1)/(T*z)
G=(0.1308*z + 0.1308)/(z**2 + 1.993*z + 1)
CONTROL=PID*G
print(CONTROL)
T=(Kd*(z - 1)/(T*z) + Ki*T*z/(z - 1) + Kp)
print(sympy.pretty(T))
I=(0.1308*z + 0.1308)*T
print(sympy.pretty(I))
K=I/(z**2 + 1.993*z + 1)
print(K.simplify())