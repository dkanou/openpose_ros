import rospy
from visualization_msgs.msg import Marker
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

pub = rospy.Publisher('wrist_traj', Marker, queue_size=10)
coords = []

def callback(msg):
    global coords
    # get coordinates
    x,y,z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    p = Point()
    p.x, p.y, p.z = x,y,z


    # remove the oldest coord
    if len(coords) > 5:
        coords = coords[1:] + [p]
    else:
        coords.append(p)

    # create marker
    spheres = Marker()

    # fill marker
    spheres.header.frame_id = "/my_frame"
    spheres.header.stamp = rospy.Time.now()
    spheres.ns = "spheres"
    spheres.action= Marker.ADD
    spheres.pose.orientation.w= 1.0
    spheres.id = 0
    spheres.type = Marker.SPHERE_LIST

    # POINTS markers use x and y scale for width/height respectively
    spheres.scale.x = 0.1
    spheres.scale.y = 0.1
    spheres.scale.z = 0.1

    # Points are green
    spheres.color.r = 1.0
    spheres.color.a = 1.0

    spheres.points = coords

    # publish
    pub.publish(spheres)


# Create rospy node, subscriber, and spin
rospy.init_node('3danimation', anonymous=True)
sub = rospy.Subscriber('/w_T_right_ee_ref', PoseStamped, callback)
rospy.spin()