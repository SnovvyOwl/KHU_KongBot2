import control
import numpy as np


            #  452.2 s + 5781
    # --------------------------------
    # s^3 + 38.85 s^2 + 851.1 s + 5774
num= np.array([452.2,5781])
den=np.array([1,38.85,851.1,5774])
sys=control.TransferFunction(num,den)
print(control.c2d(sys,0.1,method='bilinear'))