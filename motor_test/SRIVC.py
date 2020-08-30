""" THIS CODE IS Written by SeongWon Lee 
% THIS CODE IS based Practical Modeling and System Identification of R/C
% Servo Motors( 18th IEEE International Conference on Control Applications
% Part of 2009 IEEE Multi-conference on Systems and Control
% Saint Petersburg, Russia, July 8-10, 2009)

Dependency: python Control Sysytem Library
            numpy
            scipy
            matplotlib

"""
import control
import matplotlib as plt
import numpy as np
import os
os.chdir(os.getcwd())
y= np.loadtxt("KHU_KongBot2/motor_test/RefinedY.txt",delimiter=",")
u= np.loadtxt("KHU_KongBot2/motor_test/RefinedU.txt",delimiter=",")
#DP CONTROLLER
n=4
theta=np.array([1,1,1,1,1])
M=1000
j=1
flag=1
A=theta[1:]
B=theta[0]
m=1
phi=np.zeros((n+m-1,15387))
phi_hat=np.zeros((n+m-1,15387))
while ((flag==1) and (j<=2)):
    for i in range(n-1):
        p_i=np.zeros(n)
        p_i[i]=1
        print(p_i)
        Bp=control.tf(B,1)
        filter_y=control.tf(p_i,A)
        estimate_y=Bp*filter_y
        filter_y=control.tf2io(filter_y)
        estimate_y=control.tf2io(estimate_y)
        yf=control.input_output_response(filter_y,y[:,0],y[:,1])
        yf_hat=control.input_output_response(estimate_y,y[:,0],y[:,1])
        phi[i]=-yf[1]
        phi_hat[i]=-yf_hat[1]

    for i in range(m):
        p_i=np.zeros(m)
        p_i[i]=1
        filter_u=control.tf(p_i,A)
        filter_u=control.tf2io(filter_u)
        uf=control.input_output_response(filter_u,u[:,0],u[:,1])
        phi[n+i-1]=uf[1]
        print(phi)
        phi_hat[n+i-1]=uf[1]

    front=phi_hat*phi
    j=j+1








#B=np.reshape(1,n)
#s=control.TransferFunction.s
#sys=control.tf(B,A)
#sys=control.tf2io(sys)
#sys=B[0]/(A[0]*s**3+A[1]*s**2+A[2]*s+A[3])
#u.sort(axis=0)
#yf=control.input_output_response(sys,u[:,0],u[:,1])
#print(yf)
