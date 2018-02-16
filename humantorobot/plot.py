
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
from matplotlib.mlab import griddata
import matplotlib.animation as animation
import numpy as np
from math import log

def animate(i):
    ax.plot([x0[i]],[y0[i]],[z0[i]], marker='o-', c='b')
    return ax # update the data

data0, x0, y0, z0 = [],[],[],[]
data1, x1, y1, z1 = [],[],[],[]


data0 = np.genfromtxt('/data/emily/dkanou/openpose_ros/src/openpose_ros/humantorobot/RWrist.txt',delimiter=' ', dtype = None)
data1 = np.genfromtxt('/data/emily/dkanou/openpose_ros/src/openpose_ros/humantorobot/RElbow.txt',delimiter=' ', dtype = None)

for i in data0:
    #x coord
    x0 = [i.split(",", 2)[0] for i in data0]
    #y coord
    y0 = [i.split(",", 2)[1] for i in data0]
    #z coord
    z0 = [i.split(",", 2)[2] for i in data0]

for i in range(0,np.size(x0)):
    x0[i] = float(x0[i])
for i in range(0,np.size(y0)):
    y0[i] = float(y0[i])
for i in range(0,np.size(z0)):
    z0[i] = float(z0[i])

for i in data1:
    #x coord
    x1 = [i.split(",", 2)[0] for i in data1]
    #y coord
    y1 = [i.split(",", 2)[1] for i in data1]
    #z coord
    z1 = [i.split(",", 2)[2] for i in data1]

for i in range(0,np.size(x0)):
    x1[i] = float(x1[i])
for i in range(0,np.size(y0)):
    y1[i] = float(y1[i])
for i in range(0,np.size(z0)):
    z1[i] = float(z1[i])

#line equation
 
fig = plt.figure()
 
# Plot rate surface
print "Plotting 3D path ..."
#fig = plt.figure()

fig.suptitle('Right_wrist and elbow XYZ',fontsize=20)
#a0 = fig.gca(projection='3d') #use these for forearm lines
#a1 = fig.gca(projection='3d')
a0 = fig.add_subplot(111, projection = '3d') #use this for single line of path
#for i in range(10):
    #ax.plot([x1[i],x0[i]], [y1[i], y0[i]],[z1[i],z0[i]])
a0.plot_wireframe(x0,y0,z0)
#a1.plot3D(x1,y1,z1, 'red')
#ani = animation.FuncAnimation(fig, animate, np.arange(0,200), interval=250)
plt.show()