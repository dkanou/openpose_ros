


__author__ = "Brian Delhaisse"
__email__ = "Brian.Delhaisse@iit.it","Emily-Jane.Rolley@iit.it"
__credits__ = ["Brian Delhaisse", "Emily Rolley-Parnell"]
__version__ = "2.0.0"
__date__ = "21/11/2017"
__status__="Development"


#import libraries
import rospy
import numpy as np

#import publishers, subscribers and messages
from openpose_ros_msgs.msg import PersonDetection_3d
from comanpublisher import ComanPublisher

#from urdf_parser_py.urdf import URDF
import kdl_parser_py.urdf as KDLParserURDF
import PyKDL as kdl


###################
# Load URDF Model #
###################
#Path to the urdf file

coman_urdf = '/data/emily/robot_catkin_ws/src/IIT_ROBOT/coman/coman_urdf/urdf/coman.urdf'

#ROS rate
ros_rate = 100

#load urdf model
kdl_model = KDLParserURDF.treeFromFile(coman_urdf)

if kdl_model[0]:
    kdl_model = kdl_model[1]
else:
    raise ValueError("Error during the parsing")


#####################
# Chain, FK, and IK #
#####################

#define the kinematic chain
chain_RHand = kdl_model.getChain('DWYTorso', 'RForearm')

#define the Forward Kinematic solver
FK_RHand = kdl.ChainFkSolverPos_recursive(chain_RHand)

#define the Inverse Kinematic solver
IK_RHand = kdl.ChainIkSolverPos_LMA(chain_RHand)

##############
# Gazebo/ROS #
##############

#Define arm
Rarm_joints = ['RShSag', 'RShLat', 'RShYaw', 'RElbj', 'RForearmPlate']


#create publisher
if 'publisher' not in globals():
    publisher = ComanPublisher()

#define ROS rate
r = rospy.Rate(ros_rate)

#initialise q_solved
q_solved = np.array([0, 0, 0, 0, 0, 0, 0,])

def callback (msg):
    
    if(msg.num_people_detected == 1):

        #Only publish when exactly one person is detected 
        print("Exactly one person detected")

        global q_solved

        ############
        # Solve IK #
        ############

        #initial joint positions
        q_init = kdl.JntArray(chain_RHand.getNrOfJoints())
        
        for i in q_solved:
            q_init[i] = i

        q_solved = kdl.JntArray(chain_RHand.getNrOfJoints())


        
        if (msg.right_wrist.x == "nan"):
            print("No right wrist detected")
            
        else:    
            print(msg.person_ID)       
            x, y, z, = msg.right_wrist.x, msg.right_wrist.y, msg.right_wrist.z #Assign x, y and z with the coordinates from right_wrist
        


        #desired final cartesian position
        posVec = kdl.Vector(x, y, z)
        F_des = kdl.Frame(posVec)
        IK_RHand.CartToJnt(q_init, F_des, q_solved)

        #convert to np (for purposes of printing)
        q_solved = np.array([q_solved[i] for i in range(q_solved.rows())])
        
        #publish and print desired final joint positions
        publisher.send({jnt: q_solved[i] for i,jnt in enumerate(Rarm_joints)})
        print(q_solved)
        
    elif(msg.num_people_detected == 0):
        print("No person detected") 


    else:
        print("Too many people detected!", msg.num_people_detected)


Sub=rospy.Subscriber("/openpose_ros/skeleton_3d/detected_poses_keypoints_3d", PersonDetection_3d, callback)

rospy.spin()
