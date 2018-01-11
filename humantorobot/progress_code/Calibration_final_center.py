#Initial Calibration sequence
import numpy as np
import rospy
from openpose_ros_msgs.msg import PersonDetection_3d #import x, y, z
from geometry_msgs.msg import PoseStamped
import time
from scipy import interpolate

#print('hello')
body_parts = ["nose", "neck","right_shoulder","right_elbow","right_wrist","left_shoulder",
              "left_elbow","left_wrist","right_hip","right_knee","right_ankle","left_hip",
              "left_knee","left_ankle","right_eye","left_eye","right_ear","left_ear","chest"]
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
H_human = {name: (0,0,0) for name in body_parts}


#Initial calibration sequence
#for 4 seconds
#Make human move to T, N and F positions

counter = 0
n_counter = 10

name = 'right_wrist'

print('creating publisher')
pub = rospy.Publisher('/w_T_right_ee_ref', PoseStamped, queue_size=10)
    
x_RRHN, x_RRHF, x_RRHT = 0.437191, 0.973348, 0.630994
y_RRHN, y_RRHF, y_RRHT = -0.369656, -0.238957, -0.830075
z_RRHN, z_RRHF, z_RRHT = 0.638777, 1.38092, 1.34539
sx, sy, sz = 1, 1, 1

torso_2_x = 0.2
torso_2_y = 0
torso_2_z = 1.1904

T_calib_bool, N_calib_bool, F_calib_bool, H_calib_bool = False,False,False,False
robot_out = PoseStamped() #robot co-ordinates to be published
depth_phase_done = True
depth_calib = []
sx_list, sy_list, sz_list = [], [], []
depth = 1.5


class Linear(object):
    def __init__(self, a):
        m,b = a
        self.m = m
        self.b = b
    def get(self, d):
        return self.m * d + self.b

def calibration_callback(msg):
    global counter, T_calib_bool, N_calib_bool, F_calib_bool, H_calib_bool,\
           robot_out, sx, sy, sz, depth_phase_done, depth, x_HRHN, x_HRHF, \
           x_HRHT, y_HRHN, y_HRHF, y_HRHT, z_HRHN, z_HRHF, z_HRHT
    name = 'right_wrist'
    step = 0.6
    m = getattr(msg, name)
    
    if(msg.num_people_detected == 1):    

        # TODO: take a small step to the new value instead of replacing the old one
        left_shoulder = getattr(msg, 'left_shoulder')
        right_shoulder = getattr(msg, 'right_shoulder')
        if not np.isnan(left_shoulder.z) and not np.isnan(right_shoulder.z):
            depth = (left_shoulder.z + right_shoulder.z)/2.
            #depth = depth + step * (left_shoulder.z + right_shoulder.z)/2. 
            print('depth: ', depth)

        #print(counter)

        # Calibration: T pose
        if counter < n_counter:
            #collect x,y,z data and average over 4 seconds of samples
            if T_calib_bool == False:
                print("Stand in the T pose.")
                raw_input("Ready?")
                print("Ok! Stand still for 5 seconds")
                T_calib_bool = True

            #average data
            NanBool = np.isnan(m.x)
            print(m)
            if not NanBool:
                T_human[name] = ((T_human[name][0] + step*(m.x - T_human[name][0])),
                                 (T_human[name][1] + step*(m.y - T_human[name][1])),
                                 (T_human[name][2] + step*(m.z - T_human[name][2])))    
                counter += 1
            else:
                rospy.loginfo("There was a problem... Could not locate (Returned NAN)")
                    

        elif n_counter <= counter < 2*n_counter:
            if N_calib_bool == False:
                print("Stand in the Neutral pose.")
                raw_input("Ready?")
                print("Ok! Stand still for 5 seconds")
                N_calib_bool = True

            #average data
            NanBool = np.isnan(m.x)
            print(m)
            if not NanBool:
                N_human[name] = ((N_human[name][0] + step*(m.x - N_human[name][0])),
                                 (N_human[name][1] + step*(m.y - N_human[name][1])),
                                 (N_human[name][2] + step*(m.z - N_human[name][2])))    
                counter += 1   
            else:
                rospy.loginfo("There was a problem... Could not locate (Returned NAN)")

            
        elif 2*n_counter <= counter < 3*n_counter:
            if F_calib_bool == False:
                print("Stand with your arms out in front of you.")
                raw_input("Ready?")
                print("Ok! Stand still for 5 seconds")
                F_calib_bool = True

            #average data
            NanBool = np.isnan(m.x)
            print(m)
            if not NanBool:
                F_human[name] = ((F_human[name][0] + step*(m.x - F_human[name][0])),
                                 (F_human[name][1] + step*(m.y - F_human[name][1])),
                                 (F_human[name][2] + step*(m.z - F_human[name][2])))  
                counter += 1
            else:
                rospy.loginfo("There was a problem... Could not locate (Returned NAN)") 
            
            
        elif 3*n_counter <= counter < 4*n_counter:
            if H_calib_bool == False:
                #set robot to home position here!!!!
                print("Now please move to home position!")
                raw_input("Ready?")
                print("Ok! Stand still for 5 seconds")
                H_calib_bool = True

            #average data
            NanBool = np.isnan(m.x)
            print(m)
            if not NanBool:
                H_human[name] = ((H_human[name][0] + step*(m.x - H_human[name][0])),
                                 (H_human[name][1] + step*(m.y - H_human[name][1])),
                                 (H_human[name][2] + step*(m.z - H_human[name][2])))  
                counter += 1
            else:
                rospy.loginfo("There was a problem... Could not locate (Returned NAN)")

        elif counter == 4*n_counter:
            if depth_phase_done:
                counter +=1
                x_HRHN, x_HRHF, x_HRHT = N_human['right_wrist'][0] , F_human['right_wrist'][0] , T_human['right_wrist'][0]
                y_HRHN, y_HRHF, y_HRHT = N_human['right_wrist'][1] , F_human['right_wrist'][1] , T_human['right_wrist'][1]
                z_HRHN, z_HRHF, z_HRHT = N_human['right_wrist'][2] , F_human['right_wrist'][2] , T_human['right_wrist'][2]

                # x,y,z are in the robot world
                sx = (x_RRHN - x_RRHF) / (1e-5 + z_HRHN - z_HRHF) # depth
                sy = (y_RRHN - y_RRHT)/(1e-5 + x_HRHN - x_HRHT) # left-right 
                sz = (z_RRHT - z_RRHN)/(1e-5 - y_HRHT + y_HRHN) # top-bottom 

                # record the current depth and scale values
                depth_calib.append(depth)
                sx_list.append(sx)
                sy_list.append(sy)
                sz_list.append(sz)

                print("sx: " + str(sx) + ", sy: " + str(sy) + ", sz: " + str(sz)) 
                print("N_human: ", N_human['right_wrist'][2] , N_human['right_wrist'][0] , -N_human['right_wrist'][1])
                print("F_human: ", F_human['right_wrist'][2] , F_human['right_wrist'][0] , -F_human['right_wrist'][1])
                print("T_human: ", T_human['right_wrist'][2] , T_human['right_wrist'][0] , -T_human['right_wrist'][1])

                print('Sx_list: ', sx_list)
                print('Sy_list: ', sy_list)
                print('Sz_list: ', sz_list)
                print('depth_list: ', depth_calib)

                # Interpolate
                #sx = Linear(np.polyfit(depth_calib, sx_list, 1))
                #sy = Linear(np.polyfit(depth_calib, sy_list, 1))
                #sz = Linear(np.polyfit(depth_calib, sz_list, 1))

                print('Calibration DONE!')
                print('Sx_list: ', sx_list)
                print('Sy_list: ', sy_list)
                print('Sz_list: ', sz_list)
                print('depth_list: ', depth_calib)
                raw_input('Ready to start to teleoperate the robot?')

            else:
                x_HRHN, x_HRHF, x_HRHT = N_human['right_wrist'][0] , F_human['right_wrist'][0] , T_human['right_wrist'][0]
                y_HRHN, y_HRHF, y_HRHT = N_human['right_wrist'][1] , F_human['right_wrist'][1] , T_human['right_wrist'][1]
                z_HRHN, z_HRHF, z_HRHT = N_human['right_wrist'][2] , F_human['right_wrist'][2] , T_human['right_wrist'][2]

                # x,y,z are in the robot world
                sx = (x_RRHN - x_RRHF) / (1e-5 + z_HRHN - z_HRHF) # depth
                sy = (y_RRHN - y_RRHT)/(1e-5 + x_HRHN - x_HRHT) # left-right 
                sz = (z_RRHT - z_RRHN)/(1e-5 - y_HRHT + y_HRHN) # top-bottom 
                print("sx: " + str(sx) + ", sy: " + str(sy) + ", sz: " + str(sz)) 
                print("N_human: ", N_human['right_wrist'])
                print("F_human: ", F_human['right_wrist'])
                print("T_human: ", T_human['right_wrist'])

                # record the current depth and scale values
                depth_calib.append(depth)
                sx_list.append(sx)
                sy_list.append(sy)
                sz_list.append(sz)

                print('\n Phase 2: please move ~1m backward and redo the calibration steps')
                raw_input('Ready?')
                counter = 0
                depth_phase_done = True
                T_calib_bool, H_calib_bool, N_calib_bool, F_calib_bool = False, False, False, False

            human_coords['right_wrist'] = H_human['right_wrist']
        else:
            #Calculate robot coordinates relative to the previous location

            NanBool = np.isnan(m.x)
            if NanBool:
                rospy.loginfo("Lost right_wrist, using last position")
            
                rospy.loginfo("(nan) Robot coords for " + 'right_wrist' + " : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
                robot_out.header.stamp.secs = time.time() #rospy.Time.now()
                pub.publish(robot_out)
       
            else:
                robot_out = PoseStamped() #robot co-ordinates to be published
                RL = getattr(msg, 'left_shoulder')
                RR = getattr(msg, 'right_shoulder')
                mid = ((RL.x + RR.x) / 2. , (RL.y + RR.y) / 2. , (RL.z + RR.z) / 2.)
                #calculate the human xyz
                x, y, z = m.z, m.x, -m.y
                human_coords['right_wrist'] = x, y, z
                #scale the human xyz to robot
                scaled_coords['right_wrist'] = ((x - z_HRHN)  * sx,
                                                (y - x_HRHN) * sy,
                                                (z + y_HRHN) * sz)
                #robot_coords['right_wrist'] = (scaled_coords['right_wrist'][0] + torso_2_x, scaled_coords['right_wrist'][1] + torso_2_y, scaled_coords['right_wrist'][2] + torso_2_z) 
                robot_coords['right_wrist'] = (scaled_coords['right_wrist'][0] + x_RRHN,
                                               scaled_coords['right_wrist'][1] + y_RRHN, 
                                               scaled_coords['right_wrist'][2] + z_RRHN) 
                
                xR, yR, zR = robot_coords['right_wrist']
             
                rospy.loginfo("Human coords: "+str(human_coords['right_wrist']))    
             
                robot_out.pose.position.x = xR
                robot_out.pose.position.y = yR
                robot_out.pose.position.z = zR
                robot_out.pose.orientation.x = 0
                robot_out.pose.orientation.y = 0
                robot_out.pose.orientation.z = 0
                robot_out.pose.orientation.w = 1
            
                rospy.loginfo("Robot coords for " + 'right_wrist' + " : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
                robot_out.header.frame_id = 'right_wrist'
                robot_out.header.stamp.secs = time.time() #rospy.Time.now()
                #print(robot_out)
                #print(type(robot_out))
                pub.publish(robot_out)
    else:
        rospy.loginfo("Multiple people detected, waiting for the area to be cleared")            
        
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

