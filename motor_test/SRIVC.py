import control
import matplotlib as plt
import numpy as np
import os
os.chdir(os.getcwd())
y= np.loadtxt("KHU_KongBot2/motor_test/RefinedY.txt",delimiter=",")
u= np.loadtxt("KHU_KongBot2/motor_test/RefinedU.txt",delimiter=",")
