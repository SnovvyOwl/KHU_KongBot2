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
yr=y
ur=u
t=y[0:3000,0]
y=y[0:3000,1]
u=u[0:3000,1]

#DP CONTROLLER
n=3
m=0
theta=np.array([45.29,1413,17970,18130]) #INITIAL ESTIMATE

phi=np.zeros((n+m+1,len(u)))
phi_hat=np.zeros((n+m+1,len(u)))
j=1
while (j<=2):
    A=theta[0:n]
    A=np.insert(A,0,1)
    B=theta[n]
    for i in range(n):
        p_i=np.zeros(n+1)
        p_i[i+1]=1
        Bp=control.tf(B,1)
        filter_y=control.tf(p_i,A)
        estimate_yf=Bp*filter_y
        filter_y=control.tf2io(filter_y)
        estimate_yf=control.tf2io(estimate_yf)
        yf=control.input_output_response(filter_y,t,y)
        yf_hat=control.input_output_response(estimate_yf,t,y)
        phi[i]=-yf[1]
        phi_hat[i]=-yf_hat[1]

    for i in range(m+1):
        p_i=np.zeros(m+1)
        p_i[i]=1
        filter_u=control.tf(p_i,A)
        filter_u=control.tf2io(filter_u)
        uf=control.input_output_response(filter_u,t,u)
        phi[n+i]=uf[1]
        phi_hat[n+i]=uf[1]
    
    front=np.matmul(phi_hat,phi.T)
    back=np.matmul(phi,yf[1].T)
    new_theta=np.matmul(np.linalg.inv(front),back)
    theta=new_theta
    j=j+1

A=theta[0:n]
A=np.insert(A,0,1)
B=theta[n]
sys=control.tf(B,A)
sys=control.tf2io(sys)
#est_y=control.input_output_response(sys,t,u)
#plt.plot(est_y[0],est_y[1])
#plt.show()
print("end")