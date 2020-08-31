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
from matplotlib import pyplot as plt
import numpy as np
import os
os.chdir(os.getcwd())
y= np.loadtxt("KHU_KongBot2/motor_test/RefinedY.txt",delimiter=",")
u= np.loadtxt("KHU_KongBot2/motor_test/RefinedU.txt",delimiter=",")
y=y[0:3000]
u=u[0:3000]
print(u[:,1])
#DP CONTROLLER
n=3
theta=np.array([45.29,1413,17970,18130])
M=1000
j=1
flag=1
m=0
phi=np.zeros((n+m+1,len(u)))
phi_hat=np.zeros((n+m+1,len(u)))
#plt.plot(u[:,0],u[:,1])
#plt.show()print(y)
while (j<=2):
    A=theta[0:n]
    A=np.insert(A,0,1)
    B=theta[n]
    for i in range(n):
        p_i=np.zeros(n+1)
        p_i[i]=1
        Bp=control.tf(B,1)
        filter_y=control.tf(p_i,A)
        estimate_yf=Bp*filter_y
        filter_y=control.tf2io(filter_y)
        estimate_yf=control.tf2io(estimate_yf)
        yf=control.input_output_response(filter_y,y[:,0],y[:,1])
        yf_hat=control.input_output_response(estimate_yf,y[:,0],y[:,1])
        phi[i]=-yf[1]
        phi_hat[i]=-yf_hat[1]

    for i in range(m+1):
        p_i=np.zeros(m+1)
        p_i[i]=1
        filter_u=control.tf(p_i,A)
        filter_u=control.tf2io(filter_u)
        uf=control.input_output_response(filter_u,u[:,0],u[:,1])

        phi[n+i]=uf[1]
        phi_hat[n+i]=uf[1]
        #print(phi)
    front=np.matmul(phi_hat,phi.T)
    back=np.matmul(phi,yf[1].T)
    new_theta=np.matmul(np.linalg.inv(front),back)
    theta=new_theta
    print(theta)

    j=j+1

A=theta[0:n]
A=np.insert(A,0,1)
B=theta[n]
sys=control.tf(B,A)
sys=control.tf2io(sys)
sys=control.input_output_response(sys,u[:,0],u[:,1])
plt.plot(sys[0],sys[1])
plt.show()