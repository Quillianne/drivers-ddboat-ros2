import os
import sys
# if the drivers are in another folder
# you can add the path here 
# example : here drivers-ddboat-v2 is a subfolder
# sys.path.insert(0,os.getcwd()+'/drivers-ddboat-v2')
sys.path.insert(0,os.getcwd()+'/..')

# we only use the encoders and the propellers (motors)
import tc74_driver_v2 as tempdrv

import time
import numpy as np

temperature = tempdrv.TempTC74IO()
temp_left, temp_right = temperature.read_temp()
st = "Temperature : Left="
if not temp_left is None:
    st += "%d deg.C, Right="%(temp_left)
else:
    st += "None, Right="
if not temp_right is None:
    st += "%d deg.C"%(temp_right)
else:
    st += "None"
print (st)

