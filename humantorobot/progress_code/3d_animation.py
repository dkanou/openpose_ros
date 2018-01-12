# Plot animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


coords = []

fig = plt.figure()
ax = fig.gca(projection='3d')
plt.ion()
plt.show()

def callback(msg):
    global coords, ax, fig
    # create figure

    

    # get coordinates
    x,y,z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    
    # remove the oldest coord
    if len(coords) > 5:
        coords = coords[1:] + [(x,y,z)]

    # clear figure
    #ax.clear()

    # set labels and limits
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(min(0,x), max(3,x))
    ax.set_ylim(min(0,y), max(2,y))
    ax.set_zlim(min(0,z), max(3,z))

    # plot
    ax.plot([i[0] for i in coords],
            [i[1] for i in coords],
            [i[2] for i in coords], marker='o', c='b')
    #plt.show(block=False)
    plt.draw()


# Create rospy node, subscriber, and spin
rospy.init_node('3d_animation', anonymous=True)
sub = rospy.Subscriber('/w_T_right_ee_ref', PoseStamped, callback)
rospy.spin()