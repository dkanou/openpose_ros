import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from openpose_ros_msgs.msg import PersonDetection #import x, y, z 

body_parts = ["Nose","Neck","RShoulder","RElbow","RWrist","LShoulder","LElbow","LWrist","RHip","RKnee","RAnkle","LHip","LKnee","LAnkle","REye","LEye","REar","LEar","Background"]
modelsize = len(body_parts)

COCO = {i: s for i,s in enumerate(body_parts)}

human_coords = {}
robot_coords = {}
pub = rospy.Publisher('/humtorob/xyzhuman', Float64MultiArray, queue_size=10)
y_HN, z_HN = -65, -20 #240, -385 Y & Z coord of right hand of human in N pose
y_HT, z_HT = -225, 190 #95, -150 Y & Z coord of right hand of human in T pose
y_RN, z_RN= -0.156, -0.05 #Y & Z coord of right hand of robot in N pose
y_RT, z_RT= -0.50, 0.32 #Y & Z coord of right hand of robot in T pose
# x,y,z are in the robot world
sx = 1. # depth
sy = (y_RT - y_RN)/(y_HT - y_HN) # left-right 
sz = (z_RT - z_RN)*0.8/(z_HT - z_HN) # top-bottom


def calibration_callback(msg):

    #Take location of hips
    centrex = (msg.right_hip.x + msg.left_hip.x) / 2
    centrey = (msg.right_hip.y + msg.left_hip.y) / 2 
    #centrez = (msg.right_hip.z + msg.left_hip.z + msg.right_ear.z + msg.left_ear.z) / 4
    rospy.loginfo("Average XYZ calculated")
    name = 'right_wrist'
    #for k, name in enumerate(body_parts):
        #if name == 'RWrist':
    h = getattr(msg, 'right_wrist')
   # x,y,z = h.x - centrex, h.y - centry, h.z - centrez
    x ,y = h.x -centrex, h.y - centrey
    human_coords[name] = (x,-y) # in human frame (in image coord)
    
    #human_coords[name] = (-z,x,-y) # in human frame (in image coord)
    
    y,z  = human_coords[name]
    #x,y,z  = human_coords[name]
    rospy.loginfo("Human coords: "+str(human_coords))
    
    #scale by factor predetermined
    #xR,yR,zR = 0, sy*(y-554)-0.156, sz*(z+625)-0.05
    yR,zR = sy*(y-y_HN)+y_RN, sz*(z-z_HN)+z_RN
    yR = np.clip(yR, y_RT, y_RN)
    zR = np.clip(zR, z_RN, z_RT)
    #robot_coords[name] = (xR,yR,zR)
    robot_coords[name] = (yR,zR)

    #s = "Time: {0} XYZ Updated - Publishing..." .format(rospy.get_time())
    rospy.loginfo("Robot coords: "+str(robot_coords[name]))
    m = Float64MultiArray()
    m.data = robot_coords[name]
    pub.publish(m)
    #file.write(str(yR)+","+str(zR)+"\n")
        
def xyz_calibrate():
    rospy.init_node('CalibrationNode', anonymous=True)
    rospy.Subscriber('/openpose_ros/detected_poses_keypoints', PersonDetection, calibration_callback)
    # Publish robot xyz to be converted to joint angles
    rospy.spin()


if __name__ == '__main__':
    try:
        xyz_calibrate()
    except rospy.ROSInterruptException:
        file.close()
    


        #matrix multiplication from frame axes to robot axes

        #scale human x,y to robot x,y 

