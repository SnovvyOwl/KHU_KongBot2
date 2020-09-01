""" THIS CODE IS Written by SeongWon Lee 
% THIS CODE IS based Practical Modeling and System Identification of R/C
% Servo Motors( 18th IEEE International Conference on Control Applications
% Part of 2009 IEEE Multi-conference on Systems and Control
% Saint Petersburg, Russia, July 8-10, 2009)

Dependency: python Control Sysytem Library
            numpy
            scipy
            matplotlib
            from multiprocessing import Process, Queue

"""
import control
from matplotlib import pyplot as plt
import numpy as np
from multiprocessing import Process, Queue, freeze_support
import os


def fil_y(pi_a, t, y, yf):
    yf.put(control.input_output_response(pi_a, t, y))
    return

def er(th,nth,e):
    ap_Err=nth-th
    #ap_Err=ap_Err/th
    ap_Err=np.abs(ap_Err)
    ap_Err=np.sum(ap_Err)
    e.put(ap_Err)
    return

if __name__ == "__main__":
    freeze_support()
    print("Reading Data")
    y = np.loadtxt("KHU_KongBot2/motor_test/RefinedY.txt", delimiter=",") #linux
    u = np.loadtxt("KHU_KongBot2/motor_test/RefinedU.txt", delimiter=",") #linux
    #y = np.loadtxt("RefinedY.txt", delimiter=",") #window
    #u = np.loadtxt("RefinedU.txt", delimiter=",") #window
    #y = y[:,1]
    #u = u[:,1]
    #t = u[:,0]
    t = y[0:15000, 0]
    y = y[0:15000, 1]
    u = u[0:15000, 1]
    print("Init..")
    # DP CONTROLLER
    n = 3
    m = 0
    theta = np.array([45.29, 1413, 17970, 18130])  # INITIAL ESTIMATE
    A = theta[0:n]
    A = np.insert(A, 0, 1)
    B = theta[n]

    """
    init = control.tf(B, A)
    init = control.tf2io(init)
    while (j <= 1000000):
    first = control.input_output_response(init, t, u)
    plt.plot(first[0], first[1])
    plt.show()
    """

    result = Queue()
    phi = np.zeros((n + m + 1, len(u)))
    phi_hat = np.zeros((n + m + 1, len(u)))
    j = 1
    while (j <= 1000):
        print(j)
        for i in range(n):
            p_i = np.zeros(n + 1)
            p_i[i + 1] = -1
            sys_hat = control.tf(B, A)
            Pn_Ap = control.tf(p_i, A)  # P^n/A(p)
            Pn_Ap_IO = control.tf2io(Pn_Ap)
            pro1 = Process(target=fil_y, args=(Pn_Ap_IO, t, y, result))  # calc P^n*Y_f
            pro1.start()
            PnxXf = Pn_Ap * sys_hat  # P^n/A(p)*B(p)/A(p)
            PnxXf = control.tf2io(PnxXf)
            PnxXf = control.input_output_response(PnxXf, t, u)  # calc P^n*X_f
            PnxYf = result.get()
            pro1.join()
            phi[i] = PnxYf[1]
            phi_hat[i] = PnxXf[1]
            pro1.close()

        for i in range(m + 1):
            p_i = np.zeros(m + 1)
            p_i[i] = 1
            Pm_Ap = control.tf(p_i, A) # P^m/A(p)
            PmxUf = control.tf2io(Pm_Ap)
            PmxUf = control.input_output_response(PmxUf, t, u)
            phi[n + i] = PmxUf[1]
            phi_hat[n + i] = PmxUf[1]

        Pn = np.zeros(3)
        Pn = np.insert(Pn, 0, 1)
        yf = control.tf(Pn, A)
        yf = control.tf2io(yf)
        yf = control.input_output_response(yf, t, y)
        front = np.matmul(phi_hat, phi.T)
        back = np.matmul(phi_hat, yf[1].T)
        new_theta = np.matmul(np.linalg.inv(front), back)
        
        pro2=Process(target=er, args=(theta, new_theta, result))
        pro2.start()
        e=result.get()
        pro2.join()
        print(e)
        if e<0.0001:
            break

        j = j + 1
        theta = new_theta
        A = theta[0:n]
        A = np.insert(A, 0, 1)
        B = theta[n]
        pro2.close()

    sys = control.tf(B, A)
    sys = control.tf2io(sys)
    est_y = control.input_output_response(sys, t, u)
    plt.plot(est_y[0], est_y[1])
    plt.show()
    
    print("end")