import numpy as np
# from matplotlib import pyplot as plt
import math

y_meas = np.loadtxt("KHU_KongBot2/SRIVC/RefinedY.txt", delimiter=',')
PD_y_mod = np.loadtxt("KHU_KongBot2/SRIVC/PD_control_estY.txt")
PD_theta = np.loadtxt("KHU_KongBot2/SRIVC/PD_control_theta.txt")
DP_y_mod = np.loadtxt("KHU_KongBot2/SRIVC/DP_control_estY.txt")
DP_theta = np.loadtxt("KHU_KongBot2/SRIVC/DP_control_theta.txt")
y_meas = y_meas[0:15000, 1]
y_meas = np.array([y_meas])
y_mean = np.mean(y_meas)
a_E = y_meas - PD_y_mod[1, :]
a_y = y_meas - y_mean
DP_y_mod = np.array([DP_y_mod[1, :].T])
PD_R_T2 = 1 - (np.dot(a_E, a_E.T)) / (np.dot(a_y, a_y.T))
print("PD : ", PD_R_T2)
a_E = y_meas - DP_y_mod
a_y = y_meas - y_mean
y_model_mean = np.mean(DP_y_mod)
DP_R_T2 = 1 - (np.dot(a_E, a_E.T)) / (np.dot(a_y, a_y.T))
print(DP_y_mod.shape)
p = np.matmul(a_y.T, (DP_y_mod - y_model_mean))
print("DP : ", DP_R_T2)
print(p.shape)
print(a_y.shape)
# YIC_PD = math.log(np.dot(a_E,a_E.T)/(np.dot(a_y,a_y.T))+math.log(0.2*