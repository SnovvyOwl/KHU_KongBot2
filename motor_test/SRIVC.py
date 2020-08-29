import control
import matplotlib as plt
import numpy as np
import os
os.chdir(os.getcwd())
y= np.loadtxt("motor_test/RefinedY.txt",delimiter=",")
u= np.loadtxt("motor_test/RefinedU.txt",delimiter=",")
#DP CONTROLLER
A=np.array([1,1,1,1])
n=1
B=np.reshape(1,n)
s=control.TransferFunction.s
sys=control.tf(B,A)
sys=control.tf2io(sys)
#sys=B[0]/(A[0]*s**3+A[1]*s**2+A[2]*s+A[3])
u.sort(axis=0)
yf=control.input_output_response(sys,u[:,0])
