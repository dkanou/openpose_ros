#Calculating Euler Angles then Quaternion
import numpy as np
import rospy
import time
import PyKDL as kdl

elbow_xyz = np.array([0, 0, 0])
wrist_xyz = np.array([0, 1, 0])

def angles_from_vector(elbow_xyz,wrist_xyz):
    diff_vector = wrist_xyz - elbow_xyz #(elbow_xyz[0] - wrist_xyz[0], elbow_xyz[1] - wrist_xyz[1], elbow_xyz[2] - wrist_xyz[2])
    roll = np.arctan2(diff_vector[1], diff_vector[2])
    pitch = np.arctan2(diff_vector[2], diff_vector[0])
    yaw = np.arctan2(diff_vector[0], diff_vector[1])
    return roll, pitch, yaw


def Rot_matrix(thx, thy, thz):
    Rot_Xl = [[1,0,0],
             [0, np.cos(thx),-(np.sin(thx))],
             [0, np.sin(thx), np.cos(thx)]]
    Rot_Yl = [[np.cos(thy), 0, np.sin(thy)],
             [0, 1, 0],
             [-(np.sin(thy)), 0, np.cos(thy)]]
    Rot_Zl = [[np.cos(thz),-(np.sin(thz)), 0],
             [np.sin(thz), np.cos(thz), 0],
             [0, 0, 1]]
    Rot_X = np.array(Rot_Xl)
    Rot_Y = np.array(Rot_Yl)
    Rot_Z = np.array(Rot_Zl)
    Matrix_R = Rot_Z.dot(Rot_Y).dot(Rot_X)
    return Matrix_R

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

diff_vector = wrist_xyz - elbow_xyz
norm = np.linalg.norm(diff_vector)
diff_vector = diff_vector/norm
R = vector_to_rot(diff_vector, norm)
print(R)
x, y, z, w = get_quaternions(R)
print(" X : " + str(x) + " Y : " + str(y) + " Z : " + str(z) + " W : " + str(w))
print(x**2+y**2+z**2+w**2)
print(np.rad2deg(get_angles(diff_vector)))


#thx, thy, thz = angles_from_vector(elbow_xyz, wrist_xyz)
#End_Matrix  =  Rot_matrix(thx, thy, thz)
#(xx, xy, xz), (yx, yy, yz), (zx, zy, zz) = End_Matrix
#rot = kdl.Rotation(xx, yx, zx, xy, yy, zy, xz, yz, zz)
#rot = kdl.Rotation(0,0.5,0,1,1,0,0.3,0,0)
#x, y, z = rot.GetEulerZYX()
#print(x, y, z)
#x, y, z = rot.GetRPY()
#print(x, y, z)
#rot = kdl.Rotation(1,0,0,0,1,0,0,0,1)
# x,y,z,w = rot.GetQuaternion()
# print("X : " + str(x) + "Y : " + str(y) + "Z : " + str(z) + "W : " + str(w))
# x,y,z,w = get_quaternions(End_Matrix)
# print("X : " + str(x) + "Y : " + str(y) + "Z : " + str(z) + "W : " + str(w))

# print("Elbow: " + str(elbow_xyz))
# print("Wrist: " + str(wrist_xyz))
# print("Rotation angles: " + str(np.rad2deg(thx)) + " , " + str(np.rad2deg(thy)) + " , " + str(np.rad2deg(thz)))
# print(End_Matrix)
# print("X : " + str(x) + "Y : " + str(y) + "Z : " + str(z) + "W : " + str(w))
