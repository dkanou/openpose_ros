#Initial Calibration sequence
import numpy as np
import rospy
from openpose_ros_msgs.msg import PersonDetection_3d #import x, y, z
from geometry_msgs.msg import PoseStamped
import time

#print('hello')
body_parts = ["nose", "neck","right_shoulder","right_elbow","right_wrist","left_shoulder","left_elbow","left_wrist","right_hip","right_knee","right_ankle","left_hip","left_knee","left_ankle","right_eye","left_eye","right_ear","left_ear","chest"]
modelsize = len(body_parts)

COCO = {i: s for i,s in enumerate(body_parts)}

human_coords = {name: (0,0,0) for name in body_parts}
robot_coords = {name: (0,0,0) for name in body_parts}
diff_coords = {name: (0,0,0) for name in body_parts}
robot_prev = {'right_wrist': (0.715,-0.406,1.087)}
scaled_coords = {name: (0,0,0) for name in body_parts}
F_human = {name: (0,0,0) for name in body_parts}
N_human = {name: (0,0,0) for name in body_parts}
T_human = {name: (0,0,0) for name in body_parts}

#Initial calibration sequence
#for 4 seconds
#Make human move to T, N and F positions

counter = 0

def initial_human(msg):
    step = 0.6
    #collect x,y,z data and average over 4 seconds of samples
    print("Stand in the T pose.")
    raw_input("Ready?")
    print("Ok! Stand still for 5 seconds")
    initialise_time = time.time()
    while time.time() < (initialise_time + 1):
        name = 'right_wrist'
        #for name in body_parts:
        m = getattr(msg, name)

        #average data
        if not np.isnan(m.x):
            T_human[name] = ((T_human[name][0] + step*(m.x - T_human[name][0])), (T_human[name][1] + step*(m.y - T_human[name][1])), (T_human[name][2] +    step*(m.z - T_human[name][2])))    
        else:
            rospy.loginfo("There was a problem... Could not locate (Returned NAN)")
    
    if (np.isnan(T_human['right_wrist'][0])):
        raise rospy.loginfo("T_human average resulted in NAN and cannot be used, try again.")
        counter = 1
    else:
        rospy.loginfo("T_Shape average is: " + str(T_human['right_wrist']))
    

    print("Stand in the Neutral pose.")
    raw_input("Ready?")
    print("Ok! Stand still for 5 seconds")
    initialise_time = time.time()
    while time.time() < (initialise_time + 1):
        name = 'right_wrist'
        #for name in body_parts:
        m = getattr(msg, name)

        #average data
        if not np.isnan(m.x):
            N_human[name] = ((N_human[name][0] + step*(m.x - N_human[name][0])), (N_human[name][1] + step*(m.y - N_human[name][1])), (N_human[name][2] +    step*(m.z - N_human[name][2])))    
            
        else:
            rospy.loginfo("There was a problem... Could not locate (Returned NAN)")
    
    if (np.isnan(N_human['right_wrist'][0])):
        raise rospy.loginfo("N_human average resulted in NAN and cannot be used, try again.")
        counter = 1
    else:
        rospy.loginfo("N_Shape average is: " + str(N_human['right_wrist']))
    
    print("Stand with your arms out in front of you.")
    raw_input("Ready?")
    print("Ok! Stand still for 5 seconds")
    initialise_time = time.time()
    while time.time() < (initialise_time + 1):
        name = 'right_wrist'
        #for name in body_parts:
        m = getattr(msg, name)

        #average data
        if not np.isnan(m.x):
            F_human[name] = ((F_human[name][0] + step*(m.x - F_human[name][0])), (F_human[name][1] + step*(m.y - F_human[name][1])), (F_human[name][2] +    step*(m.z - F_human[name][2])))  
            
        else:
            rospy.loginfo("There was a problem... Could not locate (Returned NAN)") 

    if (np.isnan(F_human['right_wrist'][0])):
        raise rospy.loginfo("F_human average resulted in NAN and cannot be used, try again.")
        counter = 1
    else:
        rospy.loginfo("F_Shape average is: " + str(F_human['right_wrist']))
    
   
    #set robot to home position here!!!!
    print("Now please move to home position!")
    raw_input("Ready?")
    print("Ok! Stand still for 5 seconds")
    initialise_time = time.time()
    while time.time() < (initialise_time + 1):
        name = 'right_wrist'
        #for name in body_parts:
        m = getattr(msg, name)

            #average data
    if not np.isnan(m.x):
        F_human[name] = ((F_human[name][0] + step*(m.x - F_human[name][0])), (F_human[name][1] + step*(m.y - F_human[name][1])), (F_human[name][2] +    step*(m.z - F_human[name][2])))  
            
    else:
            rospy.loginfo("There was a problem... Could not locate (Returned NAN)") 

    raw_input("Press any key to start")

print('creating publisher')
pub = rospy.Publisher('/w_T_right_ee_ref', PoseStamped, queue_size=10)
    
x_RRHN, x_RRHF, x_RRHT = 0.437191, 0.973348, 0.630994
y_RRHN, y_RRHF, y_RRHT = -.369656, -0.238957, -0.830075
z_RRHN, z_RRHF, z_RRHT = 0.638777, 1.38092, 1.34539

x_HRHN, x_HRHF, x_HRHT = N_human['right_wrist'][0] , F_human['right_wrist'][0] , T_human['right_wrist'][0]
y_HRHN, y_HRHF, y_HRHT = N_human['right_wrist'][1] , F_human['right_wrist'][1] , T_human['right_wrist'][1]
z_HRHN, z_HRHF, z_HRHT = N_human['right_wrist'][2] , F_human['right_wrist'][2] , T_human['right_wrist'][2]

# x,y,z are in the robot world
sx = (x_RRHN - x_RRHF) / (1e-5 + x_HRHN - x_HRHF) # depth
sy = (y_RRHN - y_RRHT)/(1e-5 + y_HRHN - y_HRHT) # left-right 
sz = (z_RRHT - z_RRHN)/(1e-5 + z_HRHT - z_HRHN) # top-bottom


def calibration_callback(msg):
    global counter
    #print(counter)
    if counter < 1:
        counter += 1
        initial_human(msg)
        

    else:   
        
        #Calculate robot coordinates relative to the previous location
        h = getattr(msg, 'right_wrist')
        
        robot_out = PoseStamped() #robot co-ordinates to be published

        if not np.isnan(h.x):

            rospy.loginfo("Lost right_wrist, using last position")
        
            rospy.loginfo("(nan) Robot coords for " + 'right_wrist' + " : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
            robot_out.header.stamp = time.time() #rospy.Time.now()
       
            pub.publish(robot_out)
   
        else:
            robot_coords['right_wrist'] = robot_prev['right_wrist']
            diff_coords['right_wrist'] = (human_coords['right_wrist'][0] - h.x, human_coords['right_wrist'][1] - h.y, human_coords['right_wrist'][2] - h.z)
            human_coords['right_wrist'] = (h.x, h.y, h.z)
            x = diff_coords['right_wrist'][0] * sx
            y = diff_coords['right_wrist'][1] * sy
            z = diff_coords['right_wrist'][2] * sz
            scaled_coords['right_wrist'] = (-z , x, -y)
            robot_prev['right_wrist'] = (robot_coords['right_wrist'][0] + scaled_coords['right_wrist'][0], robot_coords['right_wrist'][1] + scaled_coords['right_wrist'][1], robot_coords['right_wrist'][2] + scaled_coords['right_wrist'][2])

            rospy.loginfo("Human coords: "+str(human_coords['right_wrist']))	
         
	    xR, yR, zR = robot_prev['right_wrist']
            robot_out.pose.position.x = xR
            robot_out.pose.position.y = yR
            robot_out.pose.position.z = zR
            robot_out.pose.orientation.x = 0
            robot_out.pose.orientation.y = 0
            robot_out.pose.orientation.z = 0
            robot_out.pose.orientation.w = 1
        
            rospy.loginfo("Robot coords for " + 'right_wrist' + " : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
            robot_out.header.frame_id = 'right_wrist'
            robot_out.header.stamp = time.time() #rospy.Time.now()
       
            pub.publish(robot_out)
    
        
def xyz_calibrate():
    print('creating node')
    rospy.init_node('CalibrationNode', anonymous=True)
    print('creating subscriber')
    rospy.Subscriber('/openpose_ros/skeleton_3d/detected_poses_keypoints_3d', PersonDetection_3d, calibration_callback)
    # Publish robot xyz to be converted to joint angles
    rospy.spin()


if __name__ == '__main__':
    try:
        print('calibrate')
        xyz_calibrate()
        print('end')
    except rospy.ROSInterruptException:
        file.close()

