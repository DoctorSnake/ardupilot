import numpy as np
import csv
#with open('quad_log_1.csv') as csvfile:
#    reader = csv.DictReader(csvfile)
    #for row in reader:
   #    print(row);
        
#print(reader)

#mat = matrix(reader)

#mat = np.loadtxt(open("quad_log_1.csv", "rb"), delimiter=",", skiprows=1)
mat = np.genfromtxt(open("quad_log_1.csv", "rb"), delimiter=",")



import matplotlib.pyplot as plt

rcroll = mat[:,0]
rcpitch = mat[:,1]
rcthrottle = mat[:,2]
rcyaw = mat[:,3]

roll = mat[:,4]
pitch = mat[:,5]
yaw = mat[:,6]

mot_fl = mat[:,7]
mot_bl = mat[:,8]
mot_fr = mat[:,9]
mot_br = mat[:,10]

time =  mat[:,11]



plt.figure(1)
plt.subplot(2,2,1)
plt.plot(time,rcroll)

plt.subplot(2,2,2)
plt.plot(time,rcpitch)

plt.subplot(2,2,3)
plt.plot(time,rcyaw)

plt.subplot(2,2,4)
plt.plot(time,rcthrottle)
plt.show()


plt.figure(2)
plt.subplot(3,1,1)
plt.plot(time,roll)

plt.subplot(3,1,2)
plt.plot(time,pitch)

plt.subplot(3,1,3)
plt.plot(time,yaw)

plt.show()

