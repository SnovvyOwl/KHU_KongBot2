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


def fil_y(pi_a, time, y_val, fil_y_val):
    fil_y_val.put(control.input_output_response(pi_a, time, y_val))
    return


def er(th, nth, ans):
    ap_err = nth - th
    ap_err = np.abs(ap_err)
    ap_err = ap_err/np.abs(th)
    ap_err = np.sum(ap_err)
    ans.put(ap_err)
    return


if __name__ == "__main__":
    freeze_support()
    print("Reading Data")
    #y = np.loadtxt("KHU_KongBot2/SRIVC/RefinedY.txt", delimiter=",")  # linux
    #u = np.loadtxt("KHU_KongBot2/SRIVC/RefinedU.txt", delimiter=",")  # linux
    y = np.loadtxt("SRIVC/RefinedY.txt", delimiter=",")  # window
    u = np.loadtxt("RefinedU.txt", delimiter=",")  # window
    #y = np.loadtxt("totestY.txt", delimiter=",")  # window
    #u = np.loadtxt("totestu.txt", delimiter=",")  # window
    # y = y[:,1]
    # u = u[:,1]
    t = u[0:15000, 0]
    #t = np.arange(0, 13.001, step=0.001)
    #t=t.T
    y = y[0:15000, 1]
    u = u[0:15000, 1]
    #print(t)
    #u[2074:3344]=0.4
    #u[4676:6013]=0.2
    #u[8865:10175]=16.95
    #u[10176:11439]=-0.2
    #u[12701:]=-0.2
    #plt.plot(t, u, t, y)
    #plt.show()
    print("Init..")

    # DP CONTROLLER
    #n = 3
    #m = 0
    # PD CONTROLLER
    n = 3
    m = 1
    result = Queue()
    phi = np.zeros((n + m + 1, len(u)))
    
    phi_hat = np.zeros((n + m + 1, len(u)))
    A = np.array([1, 37, 1150, 14000])  # INITIAL ESTIMATE

    for i in range(n):
        p_i = np.zeros(n)
        p_i[i] = 1
        F = control.tf(p_i, A)  # P^n/A(p)
        F = control.tf2io(F)
        Yf = control.input_output_response(F, t, y)
        phi[i] = -Yf[1].T
    for i in range(m + 1):
        p_i = np.zeros(m + 1)
        p_i[i] = 1
        Uf = control.tf(p_i, A)  # P^m/A(p)
        Uf = control.tf2io(Uf)
        Uf = control.input_output_response(Uf, t, u)
        phi[n + i] = Uf[1].T

    Pn = np.zeros(n + 1)
    Pn[0] = 1
    yfn = control.tf(Pn, A)
    yfn = control.tf2io(yfn)
    yfn = control.input_output_response(yfn, t, y)
    front = np.matmul(phi, phi.T)
    back = np.matmul(phi, yfn[1])
    theta = np.matmul(np.linalg.inv(front), back)
    A = theta[0:n]
    A = np.insert(A, 0, 1)
    B = theta[n:]
    print("Starting SRIVC...")
    j = 1
    while j <= 500:
        Bp_Ap = control.tf(B, A)
        for i in range(n):
            p_i = np.zeros(n)
            p_i[i] = 1
            F = control.tf(p_i, A)  # P^n/A(p)
            F_IO = control.tf2io(F)
            pro1 = Process(target=fil_y, args=(F_IO, t, y, result))  # calc Y_f
            pro1.start()
            Xf = F * Bp_Ap  # F(p)*B(p)/A(p)
            Xf = control.tf2io(Xf)
            Xf = control.input_output_response(Xf, t, u)  # calc P^n*X_f
            Yf = result.get()
            pro1.join()
            phi[i] = -Yf[1].T
            phi_hat[i] = -Xf[1].T
            pro1.close()

        for i in range(m + 1):
            p_i = np.zeros(m + 1)
            p_i[i] = 1
            Uf = control.tf(p_i, A)  # P^m/A(p)
            Uf = control.tf2io(Uf)
            Uf = control.input_output_response(Uf, t, u)
            phi[n + i] = Uf[1].T
            phi_hat[n + i] = Uf[1].T

        Pn = np.zeros(n + 1)
        Pn[0] = 1
        yfn = control.tf(Pn, A)
        yfn = control.tf2io(yfn)
        yfn = control.input_output_response(yfn, t, y)
        front = np.matmul(phi_hat, phi.T)
        back = np.matmul(phi_hat, yfn[1])
        new_theta = np.matmul(np.linalg.inv(front), back)
        pro2 = Process(target=er, args=(theta, new_theta, result))
        pro2.start()
        e = result.get()
        pro2.join()
        print("{} : {}".format(j,e))
        if e < 0.004:
            break
        j = j + 1
        theta = new_theta
        A = theta[0:n]
        A = np.insert(A, 0, 1)
        B = theta[n:]
        pro2.close()

    sys = control.tf(B, A)
    sys = control.tf2io(sys)
    est_y = control.input_output_response(sys, t, u)
    print("saving")
    fig = plt.figure()
    plt.plot(t, u, color="b")
    plt.plot(t, y, color='r')
    plt.plot(est_y[0], est_y[1], color="g")
    fig.savefig("Graph.png", dpi=480)
    np.savetxt("estY.txt",est_y)
    np.savetxt('theta.txt', theta)
    print("end")
