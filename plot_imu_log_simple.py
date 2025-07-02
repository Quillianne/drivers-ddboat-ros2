import matplotlib.pyplot as plt
import numpy as np 


def get_raw_data (file_name):
    fp = open (file_name)
    m = []
    for stl in fp.readlines():
        stv = stl[0:-1].split(';')
        v = []
        for i in range(len(stv)):
            v.append(float(stv[i]))
        m.append(v)

    raw = np.asarray(m)
    return raw

raw_0000 = get_raw_data ("imu_log.txt")
plt.figure(1)
plt.plot (raw_0000[:,0],raw_0000[:,4],'*')

plt.figure(2)
plt.plot (raw_0000[:,0],raw_0000[:,5],'*')

plt.figure(3)
plt.plot (raw_0000[:,0],raw_0000[:,6],'*')

plt.show()