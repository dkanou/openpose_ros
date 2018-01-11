#Initial Calibration sequence
import numpy as np
import rospy
from openpose_ros_msgs.msg import PersonDetection_3d #import x, y, z
from geometry_msgs.msg import PoseStamped
import time
from scipy import interpolate
import PyKDL as kdl

#print('hello')
body_parts = ["nose", "neck","right_shoulder","right_elbow","right_wrist","left_shoulder",
              "left_elbow","left_wrist","right_hip","right_knee","right_ankle","left_hip",
              "left_knee","left_ankle","right_eye","left_eye","right_ear","left_ear","chest"]
modelsize = len(body_parts)

COCO = {i: s for i,s in enumerate(body_parts)}

torso_2_x = 0.2
torso_2_y = 0
torso_2_z = 1.1904

human_coords = {name: (0,0,0) for name in body_parts}
robot_coords = {name: (0,0,0) for name in body_parts}
diff_coords = {name: (0,0,0) for name in body_parts}
robot_prev = {'right_wrist': (0.715 - torso_2_x,-0.406 - torso_2_y ,1.087 - torso_2_z),
              'left_wrist' : (0.715 - torso_2_x, 0.406 - torso_2_y ,1.087 - torso_2_z)}
scaled_coords = {name: (0,0,0) for name in body_parts}
position = {name: (0,0,0) for name in body_parts}
quaternion = {name: (0,0,0) for name in body_parts}
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


print('creating publishers')
pubR = rospy.Publisher('/w_T_right_ee_ref', PoseStamped, queue_size=10)
pubL = rospy.Publisher('/w_T_left_ee_ref', PoseStamped, queue_size=10)   
x_RRHN, x_RRHF, x_RRHT = 0.437191, 0.973348, 0.630994
y_RRHN, y_RRHF, y_RRHT = -0.369656, -0.238957, -0.830075
z_RRHN, z_RRHF, z_RRHT = 0.638777, 1.38092, 1.34539
sx, sy, sz = 1, 1, 1


T_calib_bool, N_calib_bool, F_calib_bool, H_calib_bool = False,False,False,False
robot_out = PoseStamped() #robot co-ordinates to be published
depth_phase_done = True
depth_calib = []
sx_list, sy_list, sz_list = [], [], []
depth = 1.5

Rz = np.array([[0, 1, 0],
               [-1, 0, 0],
               [0, 0, 1]])

file_depth = open('depth.txt', 'w')
file_RElbow = open('RElbow', 'w')
file_RWrist = open('RWrist.txt', 'w')
file_LElbow = open('LElbow.txt', 'w')
file_LWrist = open('LWrist.txt', 'w')

def vector_to_rot(v, th):
    x, y, z = v
    R = np.array([[x**2*(1-np.cos(th))+np.cos(th), x*y*(1-np.cos(th))-z*np.sin(th), x*z*(1-np.cos(th))+y*(np.sin(th))],
                 [x*y*(1-np.cos(th))+z*np.sin(th), y**2*(1-np.cos(th))+np.cos(th), y*z*(1-np.cos(th))-x*np.sin(th)],
                 [x*z*(1-np.cos(th))-y*np.sin(th), y*z*(1-np.cos(th))+x*np.sin(th), z**2*(1-np.cos(th))+np.cos(th)]])
    return R

def get_quaternions(M):
    w = 0.5*(np.sqrt(M[0,0]+M[1,1]+M[2,2]+1))
    x = 0.5*(np.sign(M[2,1]-M[1,2])*np.sqrt(M[0,0]-M[1,1]-M[2,2]+1))
    y = 0.5*(np.sign(M[0,2]-M[2,0])*np.sqrt(M[1,1]-M[2,2]-M[0,0]+1))
    z = 0.5*(np.sign(M[1,0]-M[0,1])*np.sqrt(M[2,2]-M[0,0]-M[1,1]+1))
    return x, y, z, w

def get_angles(v):
    x, y, z = v
    r = np.sqrt(x**2 + y**2)
    if r == 0:
        alp = 0
        if z >= 0: 
            bet = 0
        else:
            bet = np.deg2rad(90)
    else:
        alp = np.arccos(x/r)
        bet = np.deg2rad(90)-np.arctan(z/r)
    return alp, bet  


class Linear(object):
    def __init__(self, a):
        m,b = a
        self.m = m
        self.b = b
    def get(self, d):
        return self.m * d + self.b

def calibration_callback(msg):
    global counter, T_calib_bool, N_calib_bool, F_calib_bool, H_calib_bool, robot_out,\
            sx, sy, sz, depth_phase_done, depth
    #name = 'right_wrist'
    step = 0.6
    m = getattr(msg, 'right_wrist')
    n = getattr(msg, 'left_wrist')
    p = getattr(msg, 'right_elbow')
    q = getattr(msg, 'left_elbow')
    if(msg.num_people_detected == 1):    

        # TODO: take a small step to the new value instead of replacing the old one
        left_shoulder = getattr(msg, 'left_shoulder')
        right_shoulder = getattr(msg, 'right_shoulder')
        if not np.isnan(left_shoulder.z) and not np.isnan(right_shoulder.z):
            depth = (left_shoulder.z + right_shoulder.z)/2.
            #depth = depth + step * (left_shoulder.z + right_shoulder.z)/2. 
            print('depth: ', depth)

        #print(counter)
        NanBoolR = np.isnan(m.x)
        NanBoolL = np.isnan(n.x)

        # Calibration: T pose
        if counter < n_counter:
            #collect x,y,z data and average over 4 seconds of samples
            if T_calib_bool == False:
                print("Stand in the T pose.")
                raw_input("Ready?")
                print("Ok! Stand still for 5 seconds")
                T_calib_bool = True

            #average data
            print(m)
            print(n)
            if not NanBoolL and not NanBoolR:
                for i,j in zip(['right_wrist','left_wrist'],[m,n]):
                    T_human[i] = ((T_human[i][0] + step*(j.x - T_human[i][0])),
                                 (T_human[i][1] + step*(j.y - T_human[i][1])),
                                 (T_human[i][2] + step*(j.z - T_human[i][2])))    
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
            print(m)
            print(n)
            if not NanBoolL and not NanBoolR:
                for i,j in zip(['right_wrist','left_wrist'],[m,n]):
                    N_human[i] = ((N_human[i][0] + step*(j.x - N_human[i][0])),
                                 (N_human[i][1] + step*(j.y - N_human[i][1])),
                                 (N_human[i][2] + step*(j.z - N_human[i][2])))   
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
            print(m)
            print(n)
            if not NanBoolL and not NanBoolR:
                for i,j in zip(['right_wrist','left_wrist'],[m,n]):
                    F_human[i] = ((F_human[i][0] + step*(j.x - F_human[i][0])),
                                 (F_human[i][1] + step*(j.y - F_human[i][1])),
                                 (F_human[i][2] + step*(j.z - F_human[i][2]))) 
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
            print(m)
            print(n)
            if not NanBoolL and not NanBoolR:
                for i,j in zip(['right_wrist','left_wrist'],[m,n]):
                    H_human[i] = ((H_human[i][0] + step*(j.x - H_human[i][0])),
                                 (H_human[i][1] + step*(j.y - H_human[i][1])),
                                 (H_human[i][2] + step*(j.z - H_human[i][2]))) 
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
                print("N_human: ", N_human['right_wrist'])
                print("F_human: ", F_human['right_wrist'])
                print("T_human: ", T_human['right_wrist'])

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
            human_coords['left_wrist'] = H_human['left_wrist']
        else:
            #Calculate robot coordinates relative to the previous location

            #if 'robot_out' not in locals():
            #    robot_out = PoseStamped() #robot co-ordinates to be published

            if NanBoolL or NanBoolR:
                rospy.loginfo("Lost right_wrist, using last position")
            
                rospy.loginfo("(nan) Robot coords for right_wrist : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
                rospy.loginfo("(nan) Robot coords for left_wrist : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
                
                robot_out.header.stamp.secs = time.time() #rospy.Time.now()
                #pub.publish(robot_out)
       
            else:
                file_depth.write(depth)
                file_LWrist.write(str(n.x)+","+str(n.y)+","+str(n.z))
                file_LElbow.write(str(q.x)+","+str(q.y)+","+str(q.z))
                file_RWrist.write(str(m.x)+","+str(m.y)+","+str(m.z))
                file_RElbow.write(str(p.x)+","+str(p.y)+","+str(p.z))

                for i,j,pub,l in zip(['right_wrist','left_wrist'],[m,n],[pubR,pubL],[p,q]):                
                    robot_out = PoseStamped() #robot co-ordinates to be published
                    robot_coords[i] = robot_prev[i]
                    diff_coords[i] = (human_coords[i][0] - j.x,
                                      human_coords[i][1] - j.y,
                                      human_coords[i][2] - j.z)
                    human_coords[i] = (j.x, j.y, j.z)
                    dx = diff_coords[i][0]# * sx#.get(depth) #sx([depth])[0]
                    dy = diff_coords[i][1]# * sy#.get(depth) #sy([depth])[0]
                    dz = diff_coords[i][2]# * sz#.get(depth) #sz([depth])[0]
                    scaled_coords[i] = (dz, -dx, dy)
                    robot_prev[i] = (robot_coords[i][0] + scaled_coords[i][0],
                                    robot_coords[i][1] + scaled_coords[i][1],
                                    robot_coords[i][2] + scaled_coords[i][2])

                    rospy.loginfo("Human coords: "+str(human_coords[i]))    

                    #Orientation Calculations
                    wrist_xyz = np.array([j.z, -j.x, j.y])
                    elbow_xyz = np.array([l.z, -l.x, l.y])
                    diff_vector = wrist_xyz - elbow_xyz
                    norm = np.linalg.norm(diff_vector)
                    diff_vector = diff_vector/norm # unit vector
                    print("Vector: ", diff_vector)
                    R = vector_to_rot(diff_vector, norm)
                    #print(Rz.dot(R))
                    x, y, z, w = get_quaternions(R)
                    print("X : " + str(x) + ", Y : " + str(y) + ", Z : " + str(z) + ", W : " + str(w))
                    
                    if i == 'right_wrist':
                        print('\n##################')
                        print(np.rad2deg(get_angles(diff_vector)))
                        print('##################\n')

                    xR, yR, zR = robot_prev[i]
                    robot_out.pose.position.x = xR
                    robot_out.pose.position.y = yR
                    robot_out.pose.position.z = zR

                    #R = vector_to_rot(np.array([1,0,0]), 1)
                    #x, y, z, w = get_quaternions(Rz.dot(R))
                
                    robot_out.pose.orientation.x = 0#x
                    robot_out.pose.orientation.y = 0#y
                    robot_out.pose.orientation.z = 0#z
                    robot_out.pose.orientation.w = 1#w
                
                    rospy.loginfo("Robot coords for " + i + " : "+str(robot_out.pose.position.x)+', '+str(robot_out.pose.position.y)+', ' +str(robot_out.pose.position.z))
                    robot_out.header.frame_id = i
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
    file_depth.close()
    file_RElbow.close()
    file_RWrist.close()
    file_LWrist.close()
    file_LElbow.close()


if __name__ == '__main__':
    try:
        print('calibrate')
        xyz_calibrate()
        print('end')
    except rospy.ROSInterruptException:
        file.close()

