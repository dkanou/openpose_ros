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
from std_msgs.msg import Float64MultiArray

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
#IKV_RHand = kdl.ChainIkSolverVel_pinv(chain_RHand)
#IK_RHand = kdl.ChainIkSolverPos_NR(chain_RHand, FK_RHand, IKV_RHand)

##############
# Gazebo/ROS #
##############

#Define arm
Rarm_joints = ['RShSag', 'RShLat', 'RShYaw', 'RElbj', 'RForearmPlate']

Enu_Rarm = {i: s for i,s in enumerate(Rarm_joints)}

#create publisher
#if 'publisher' not in globals():
publisher = ComanPublisher()

#define ROS rate
r = rospy.Rate(ros_rate)

#initialise q_solved
q_solved = np.array([0, 0, 0, 0, 0, 0, 0,])
x = 0.005305
def callback (msg):
    
# if(msg.num_people_detected == 1):

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

    print(chain_RHand)
    
    # if (msg.right_wrist.x == True): #if x = "nan"
    #     print("No right wrist detected")
        
    # else:    
    #     print(msg.person_ID)       
    #    #x, y, z, = msg.right_wrist.x, msg.right_wrist.y, msg.right_wrist.z #Assign x, y and z with the coordinates from right_wrist
    y , z = msg.data


    #desired final cartesian position
    posVec = kdl.Vector(x, y, z)
    F_des = kdl.Frame(posVec)
    IK_RHand.CartToJnt(q_init, F_des, q_solved)

    #convert to np (for purposes of printing)
    q_solved = np.array([q_solved[i] for i in range(q_solved.rows())])
    
    #smoothing
    q_res = q_solved
    
    if ((abs(q_res[i]-q_solved[i]) > 0.5) for i in enumerate(Rarm_joints)):
	q_solved[i] = q_solved[i]

    q_solved[Rarm_joints.RShSag] = np.clip(q_solved[Rarm_joints.RShSag], 0.6, -1.57)
    
    #publish and print desired final joint position
    publisher.send({jnt: q_solved[i] for i,jnt in enumerate(Rarm_joints)})
    print(q_solved)

    
#elif(msg.num_people_detected == 0):
  #  print("No person detected") 


#else:
   # print("Too many people detected!", msg.num_people_detected)


Sub=rospy.Subscriber("/humtorob/xyzhuman", Float64MultiArray, callback)

rospy.spin()

