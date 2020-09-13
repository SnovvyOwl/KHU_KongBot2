import numpy as np


def rt2(measured_y, modeled_y):
    a_e = measured_y - modeled_y[1, :]
    mean = np.mean(measured_y)
    a_y = measured_y - mean
    r2t = 1 - (np.dot(a_e, a_e.T)) / (np.dot(a_y, a_y.T))
    return r2t


y_meas = np.loadtxt("RefinedY.txt", delimiter=',')
PD_y_mod = np.loadtxt("PD_control_estY.txt")
PD_theta = np.loadtxt("PD_control_theta.txt")
DP_y_mod = np.loadtxt("DP_control_estY.txt")
DP_theta = np.loadtxt("DP_control_theta.txt")
PI_y_mod = np.loadtxt("PI_control_estY.txt")
PI_theta = np.loadtxt("PI_control_theta.txt")
PID_y_mod = np.loadtxt("PID_control_estY.txt")
PID_theta = np.loadtxt("PID_control_theta.txt")
y_meas = y_meas[0:15000, 1]
y_meas = np.array([y_meas])
print("PD : ", rt2(y_meas, PD_y_mod))
print("DP : ", rt2(y_meas, DP_y_mod))
print("PI : ", rt2(y_meas, PI_y_mod))
print("PID : ", rt2(y_meas, PID_y_mod))