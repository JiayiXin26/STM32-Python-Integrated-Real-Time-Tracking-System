import time
import pickle
import matplotlib.pyplot as plt
from matplotlib.figure import Figure




# Open the file and prepare for loading data

name="save_1736479406"  #Enter the filename

filenamne="data/"+name+".pkl"

# Load multiple variables
with open(filenamne, 'rb') as f:

    GTab_mb_imgx= pickle.load(f)
    GTab_mb_imgy= pickle.load(f)
    GTab_MBFW= pickle.load(f)
    GTab_MBFY= pickle.load(f)
    GTab_djjiao_FW= pickle.load(f)
    GTab_djjiao_FY= pickle.load(f)
    GTab_dpwm_fw= pickle.load(f)
    GTab_dpwm_fy= pickle.load(f)
    G_capMB_X= pickle.load(f)
    G_capMB_Y= pickle.load(f)
    G_capMB_Z= pickle.load(f)
    G_capMB_time= pickle.load(f)
    GTab_distance_CSB= pickle.load(f)
    GTab_distance_JG= pickle.load(f)
    GTab_Gpix_targe_area=pickle.load(f)
    GTab_pix_to_dis =pickle.load(f)


print(G_capMB_time)

#Plot and visualize the recorded data
plt.figure(1)
plt.plot(G_capMB_time,'b*')
plt.title('Time')

plt.figure(2)
plt.plot(G_capMB_time, GTab_mb_imgx)

plt.figure(3)
plt.plot(GTab_MBFW, GTab_MBFY)
plt.title('Target Azimuth and Elevation')

plt.figure(4)
plt.plot(G_capMB_time, GTab_distance_CSB)

plt.figure(5)
plt.plot(G_capMB_time, GTab_Gpix_targe_area)


plt.show()
