import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from openpose_ros_msgs.msg import PersonDetection #import x, y, z 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

body_parts = ["Nose","Neck","RShoulder","RElbow","RWrist","LShoulder","LElbow","LWrist","RHip","RKnee","RAnkle","LHip","LKnee","LAnkle","REye","LEye","REar","LEar","Background"]
modelsize = len(body_parts)

COCO = {i: s for i,s in enumerate(body_parts)}

human_coords = {}
robot_coords = {}
pub = rospy.Publisher('/humtorob/xyzhuman', Float64MultiArray, queue_size=10)

Wry_HN, Wrz_HN = 552, 627 #240, -385 Y & Z pixel of right hand of human in N pose
Wry_HT, Wrz_HT = 288, 292 #95, -150 Y & Z coord of right hand of human in T pose
Wry_RN, Wrz_RN= -0.156, -0.05 #Y & Z coord of right hand of robot in N pose
Wry_RT, Wrz_RT= -0.53, 0.32 #Y & Z coord of right hand of robot in T pose

Shy_HN, Shz_HN = 586, 336 # Y & Z coord of right shoulder of human in N pose
Shy_HT, Shz_HT = 580, 307 # Y & Z coord of right shoulder of human in T pose
Shy_RN, Shz_RN= -0.083, 0.324 #Y & Z coord of right shoulder of robot in N pose
Shy_RT, Shz_RT= -0.082, 0.324 #Y & Z coord of right shoulder of robot in T pose
Ely_HN, Elz_HN = 559, 494 # Y & Z coord of right elbow of human in N pose
Ely_HT, Elz_HT = 423, 310 # Y & Z coord of right elbow of human in T pose
Ely_RN, Elz_RN= -0.158, 0.1443 #Y & Z coord of right elbow of robot in N pose
Ely_RT, Elz_RT= -0.336, 0.321 #Y & Z coord of right elbow of robot in T pose
# x,y,z are in the robot world
sx = 1. # depth
sRShy = (Shy_RN-Shy_RT)/(Shy_HN-Shy_HT) # left-right 
sRShz = (Shz_RN-Shz_RT)/(Shz_HN-Shz_HT) # down-up
sREly = (Ely_RN-Ely_RT)/(Ely_HN-Ely_HT)
sRElz = (Elz_RN-Elz_RT)/(Elz_HN-Elz_HT)
sRWry = (Wry_RN-Wry_RT)/(Wry_HN-Wry_HT)
sRWrz = (Wrz_RN-Wrz_RT)/(Wrz_HN-Wrz_HT)


def calibration_callback(msg):

    #Take location of hips
    centrex = (msg.right_hip.x + msg.left_hip.x) / 2
    centrey = (msg.right_hip.y + msg.left_hip.y) / 2 
    #centrez = (msg.right_hip.z + msg.left_hip.z + msg.right_ear.z + msg.left_ear.z) / 4
    rospy.loginfo("Average XYZ centre calculated")
    #name = 'right_wrist'
    #for k, name in enumerate(body_parts):
        #if name == 'RWrist':
    HWr = getattr(msg, 'right_wrist')
    HEl = getattr(msg, 'right_elbow')
    HSh = getattr(msg, 'right_shoulder')
    rospy.loginfo("Wrist, Elbow and Shoulder information collected")
   # x,y,z = h.x - centrex, h.y - centry, h.z - centrez
   # x ,y = h.x -centrex, h.y - centrey
    HWrx, HWry = HWr.x - centrex, HWr.y - centrey #create centralised human coords
    HElx, HEly = HEl.x - centrex, HEl.y - centrey 
    HShx, HShy = HSh.x - centrex, HSh.y - centrey 
    rospy.loginfo("XYZ Centralised")

    #scale by predetermined factor
    RoShy, RoShz = HShx * sRShy, HShy * sRShz
    RoEly, RoElz = (HElx * sREly), (HEly * sRElz)
    RoWry, RoWrz = (HWrx * sRWry), (HWry * sRWrz)
   
    human_coords['right_wrist'] = (HWrx,-HWry) # in human frame (in image coord)
    human_coords['right_elbow'] = (HElx, -HEly)
    human_coords['right_shoulder'] = (HShx, -HShy)
    
    #human_coords[name] = (-z,x,-y) # in human frame (in image coord)
    
   # y,z  = human_coords[name]
    #x,y,z  = human_coords[name]
    rospy.loginfo("Human coords: "+str(human_coords))
    
    #robot_coords[name] = (xR,yR,zR)
    robot_coords['right_wrist'] = (RoWry, RoWrz)
    robot_coords['right_elbow'] = (RoEly, RoElz)
    robot_coords['right_shoulder'] = (RoShy, RoShz)
    #s = "Time: {0} XYZ Updated - Publishing..." .format(rospy.get_time())
    #RoWryList, RoWrzList, RoElyList, RoElzList, RoShyList, RoShzList = [],[],[],[],[],[]
    #RoWryList.append(RoWry)
    #RoWrzList.append(RoWrz)
    #RoElyList.append(RoEly)
    #RoElzList.append(RoElz)
    #RoShyList.append(RoShy)
    #RoShzList.append(RoShz)
    
   # plt.plot(RoWryList, RoWrzList, 'bs', RoElyList, RoElzList, 'r--', RoShyList, RoShzList, 'g^')
    #plt.plot(RoWry,RoWrz, 'ro-')
    
    rospy.loginfo("Robot coords: "+str(robot_coords['right_shoulder']))
    m = Float64MultiArray()
    m.data = robot_coords['right_shoulder']
    pub.publish(m)
    rospy.loginfo("Robot coords: "+str(robot_coords['right_elbow'])) 
    m = Float64MultiArray()
    m.data = robot_coords['right_elbow']
    pub.publish(m)
    #file.write(str(yR)+","+str(zR)+"\n")
    rospy.loginfo("Robot coords: "+str(robot_coords['right_wrist']))
    m = Float64MultiArray()
    m.data = robot_coords['right_wrist']
    pub.publish(m)
        
def xyz_calibrate():
    rospy.init_node('CalibrationNode', anonymous=True)
    rospy.Subscriber('/openpose_ros/detected_poses_keypoints', PersonDetection, calibration_callback)
    # Publish robot xyz to be converted to joint angles
    rospy.spin()


if __name__ == '__main__':
    try:
        xyz_calibrate()
        #plt.show()
    except rospy.ROSInterruptException:
        file.close()
    


        #matrix multiplication from frame axes to robot axes

        #scale human x,y to robot x,y 


