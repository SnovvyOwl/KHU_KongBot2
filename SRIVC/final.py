import control
import numpy as np
from matplotlib import pyplot as plt
print("Reading Data")
y = np.loadtxt("KHU_KongBot2/SRIVC/RefinedY.txt", delimiter=",")  # linux
u = np.loadtxt("KHU_KongBot2/SRIVC/RefinedU.txt", delimiter=",")  # linux
t = u[0:15000, 0]
y = y[0:15000, 1]
u = u[0:15000, 1]
theta=np.loadtxt("KHU_KongBot2/SRIVC/PD_control_theta.txt")

"""
    PD_Control

             452.2 s + 5781
    --------------------------------
    s^3 + 38.85 s^2 + 851.1 s + 5774

"""
n=3
m=1
A = theta[0:n]
A = np.insert(A, 0, 1)
B = theta[n:]
sys=control.tf(B,A)
print(sys)
sys=control.tf2io(sys)
y_m=control.input_output_response(sys,t,u)
fig = plt.figure()
plt.plot(t, u, color="b")
plt.plot(t, y, color='r')
plt.plot(y_m[0], y_m[1], color="g")
plt.show()