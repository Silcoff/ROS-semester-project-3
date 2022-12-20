import numpy as np

import tf_transformations as tf

import math
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose
import numpy as np

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose

from scipy.spatial.transform import Rotation as R

import tf_transformations as tf
from std_msgs.msg import Bool

from tf2_ros import TransformBroadcaster
import traceback

import time


def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


import numpy as np
import math as m
  
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])



def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)


    # c1 = theta1
    # s1 = theta1
    # c2 = theta2
    # s2 = theta2
    # c3 = theta3
    # s3 = theta3
    
    if order == 'xzx':
        matrix=np.array([[c2, -c3*s2, s2*s3],
                        [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                        [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
    elif order=='xyx':
        matrix=np.array([[c2, s2*s3, c3*s2],
                        [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                        [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]])
    elif order=='yxy':
        matrix=np.array([[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                        [s2*s3, c2, -c3*s2],
                        [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]])
    elif order=='yzy':
        matrix=np.array([[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                        [c3*s2, c2, s2*s3],
                        [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]])
    elif order=='zyz':
        matrix=np.array([[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                        [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                        [-c3*s2, s2*s3, c2]])
    elif order=='zxz':
        matrix=np.array([[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                        [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                        [s2*s3, c3*s2, c2]])
    elif order=='xyz':
        matrix=np.array([[c2*c3, -c2*s3, s2],
                        [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                        [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    elif order=='xzy':
        matrix=np.array([[c2*c3, -s2, c2*s3],
                        [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                        [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]])
    elif order=='yxz':
        matrix=np.array([[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                        [c2*s3, c2*c3, -s2],
                        [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]])
    elif order=='yzx':
        matrix=np.array([[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                        [s2, c2*c3, -c2*s3],
                        [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]])
    elif order=='zyx':
        matrix=np.array([[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                        [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                        [-s2, c2*s3, c2*c3]])
    elif order=='zxy':
        matrix=np.array([[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                        [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                        [-c2*s3, s2, c2*c3]])

    return matrix

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
            This rotation matrix converts a point in the local reference 
            frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                            
    return rot_matrix



def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))









################################################
### comput therotical transfer frame ###########
################################################


def theory_transfer_frame(Q,position):
    print("theory_transfer_frame Q"       , Q)    
    print("theory_transfer_frame position", position)
    in_quan_x = Q[0]
    in_quan_y = Q[1]
    in_quan_z = Q[2]
    in_quan_w = Q[3]

    in_quan = [in_quan_w,in_quan_x,in_quan_y,in_quan_z]


    in_rot_matrix = quaternion_rotation_matrix(in_quan)


    in_post_x = position[0]
    in_post_y = position[1]
    in_post_z = position[2]

    theory_tran_transfrom =  [[in_rot_matrix[0][0], in_rot_matrix[0][1], in_rot_matrix[0][2], in_post_x],
                              [in_rot_matrix[1][0], in_rot_matrix[1][1], in_rot_matrix[1][2], in_post_y],
                              [in_rot_matrix[2][0], in_rot_matrix[2][1], in_rot_matrix[2][2], in_post_z],
                              [0                  , 0                  , 0                  , 1        ]]
    return theory_tran_transfrom


################################################
### comput physical hand frame       ###########
################################################


def physical_hand_frame(Euler,position):
    print("physical_hand_frame euler: "   , Euler)
    print("physical_hand_frame poistion: ", position)
    in_post_x = position[0]/1000
    in_post_y = position[1]/1000
    in_post_z = position[2]/1000



    # in_RX = Euler[0] * 180/np.pi 
    # in_RY = Euler[1] * 180/np.pi 
    # in_RZ = Euler[2] * 180/np.pi 
    a = Euler/np.linalg.norm(Euler)

    angle =Euler/a
    r = R.from_rotvec(angle[0] * np.array(a))    
    in_rot_matrix = r.as_matrix()

    # print("angle for axis: ", angle)
    # in_rot_matrix = rotation_matrix(in_RX,in_RY,in_RZ,'zyx')
    


    physical_hand_transfrom =  [[in_rot_matrix[0][0], in_rot_matrix[0][1], in_rot_matrix[0][2], in_post_x],
                                [in_rot_matrix[1][0], in_rot_matrix[1][1], in_rot_matrix[1][2], in_post_y],
                                [in_rot_matrix[2][0], in_rot_matrix[2][1], in_rot_matrix[2][2], in_post_z],
                                [0                  , 0                  , 0                  , 1        ]]
    return physical_hand_transfrom



################################################
### comput physical transfer frame       #######
################################################


def physical_transfer_frame(physical):


    offset = [[   1   ,   0   ,   0   ,   0   ],
              [   0   ,   1   ,   0   ,   0   ],
              [   0   ,   0   ,   1   ,  -0.1   ],
              [   0   ,   0   ,   0   ,   1   ]]
    

    physical_hand_transfrom = np.dot(physical,offset)

    return physical_hand_transfrom



################################################
###         write in the test data       #######
################################################


# quan_test_xyzw = np.array([
#     [-0.453015, -0.822274, -0.239755, 0.247305], #1
# ])
# postition_xyz_for_quan = np.array([
#     [-0.439742, -0.822155,0.478032], #1
# ])


# euler_xyz = np.array([
#     [-3.137, 0.001, -2.340], #1
# ])
# postition_xyz_for_euler = np.array([
#     [-505.94, -512.85,-58.24],
# ])

quan_test_xyzw = [[]]
postition_xyz_for_quan = [[]]


euler_xyz = [[]]
postition_xyz_for_euler = [[]]




postition_xyz_for_quan.append([-0.436233, -0.462312, 0.41974]) # 1
quan_test_xyzw.append([-0.445475, -0.88682, 0.0446061, 0.114508]) # 1
postition_xyz_for_euler.append([-469.33,-503.55,352]) # 1
euler_xyz.append([1.35,2.667,-0.113]) # 1


postition_xyz_for_quan.append(  [-0.437068, -0.465518, 0.392687]   )# 2
quan_test_xyzw.append(    [-0.383504, -0.917911, 0.0156118, 0.100606]         )# 2
postition_xyz_for_euler.append(  [-460.63,-496.49,333.93]  )# 2
euler_xyz.append(       [1.178,2.768,-0.099]           )# 2


postition_xyz_for_quan.append(  [-0.446734, -0.511806, 0.426646]   )# 3
quan_test_xyzw.append(       [-0.398931, -0.91394, -0.0439143, 0.0603321]      )# 3
postition_xyz_for_euler.append(  [-477.67,-502.96,363.51]  )# 3
euler_xyz.append(    [1.297,2.627,-0.063]              )# 3



postition_xyz_for_quan.append(  [-0.464817, -0.461265, 0.418994]   )# 4
quan_test_xyzw.append(      [-0.426042, -0.902583, 0.0015884, 0.061886]       )# 4
postition_xyz_for_euler.append(  [-465.97,-471.51,350.62]  )# 4
euler_xyz.append(        [1.298,2.629,-0.067]          )# 4


postition_xyz_for_quan.append(   [-0.451967, -0.522695, 0.409894]  )# 5
quan_test_xyzw.append(         [-0.419779, -0.902472, -0.0919015, 0.0297229]    )# 5
postition_xyz_for_euler.append(  [-453.78,-514.14,347.53]  )# 5
euler_xyz.append(            [1.346,2.67,0.023]      )# 5


postition_xyz_for_quan.append( [-0.504349, -0.55202, 0.338916]    )# 6
quan_test_xyzw.append(         [0.419504, 0.793598, 0.435672, 0.0664013]    )# 6
postition_xyz_for_euler.append(  [-467.22,-517.64,313.46]  )# 6
euler_xyz.append(         [1.382,2.279,1.064]         )# 6




postition_xyz_for_quan.append(  [-0.51149, -0.574578, 0.316888]   )# 7
quan_test_xyzw.append(       [0.452242, 0.749078, 0.48381, 0.0169575]      )# 7
postition_xyz_for_euler.append(  [-496.55,-506.02,316.98]  )# 7
euler_xyz.append(      [1.627,2.068,1.119]            )# 7





postition_xyz_for_quan.append( [-0.544505, -0.501582, 0.34052]    )# 8
quan_test_xyzw.append(         [0.429304, 0.802805, 0.411525, 0.0430008]    )# 8
postition_xyz_for_euler.append(  [-483.50,-483.51,322.15]  )# 8
euler_xyz.append(         [1.462,2.339,0.83]         )# 8


postition_xyz_for_quan.append( [-0.527862, -0.557773, 0.310578]    )# 9
quan_test_xyzw.append(         [0.435295, 0.761319, 0.479394, 0.0330577]    )# 9
postition_xyz_for_euler.append(  [-481.54,-515.49,296.65]  )# 9
euler_xyz.append(          [1.555,2.143,1.012]        )# 9





postition_xyz_for_quan.append(  [-0.48498, -0.544009, 0.349942]   )# 10
quan_test_xyzw.append(         [0.42154, 0.814379, 0.398068, 0.025138]    )# 10
postition_xyz_for_euler.append(   [-464.79,-490.56,324.20] )# 10
euler_xyz.append(       [1.486,2.235,0.849]           )# 10


postition_xyz_for_quan.append(  [-0.382559, -0.450369, 0.344237]   )# 11
quan_test_xyzw.append(          [-0.21793, -0.877433, 0.315607, 0.288114]   )# 11
postition_xyz_for_euler.append(  [-408.62,-498.45,325.44]  )# 11
euler_xyz.append(         [0.629,3.24,-1.369]         )# 11






postition_xyz_for_quan.append(  [-0.414716, -0.404483, 0.31053]   )# 12
quan_test_xyzw.append(         [-0.262382, -0.84558, 0.384766, 0.260971]    )# 12
postition_xyz_for_euler.append(  [-417.33,-480.85,294.64]  )# 12
euler_xyz.append(           [0.728,3.131,-1.58]       )# 12


postition_xyz_for_quan.append(  [-0.377685, -0.44854, 0.225827]   )# 13
quan_test_xyzw.append(           [-0.172704, -0.836964, 0.360608, 0.373667]  )# 13
postition_xyz_for_euler.append(  [-427.91,-516.40,266.14]  )# 13
euler_xyz.append(                [0.503,3.342,-1.396]  )# 13

postition_xyz_for_quan.append(  [-0.392577, -0.409895, 0.311192]   )# 14
quan_test_xyzw.append(        [-0.263137, -0.859022, 0.330701, 0.288924]     )# 14
postition_xyz_for_euler.append(  [-434,-469.12,290.50]  )# 14
euler_xyz.append(              [0.604,2.977,-1.544]    )# 14


postition_xyz_for_quan.append(    [-0.418257, -0.420082, 0.349351] )# 15
quan_test_xyzw.append(           [-0.258258, -0.892333, 0.268032, 0.255349]  )# 15
postition_xyz_for_euler.append(  [-433.09,-493.64,314.43]  )# 15
euler_xyz.append(                 [0.645,3.186,-1.362] )# 15


postition_xyz_for_quan.append(   [-0.466383, -0.513924, 0.345237]  )# 16
quan_test_xyzw.append(           [0.311084, 0.828016, -0.130239, 0.447945]  )# 16
postition_xyz_for_euler.append(   [-465.97,-500.94,307.99] )# 16
euler_xyz.append(                [0.774,2.092,-0.180]  )# 16




postition_xyz_for_quan.append(   [-0.519585, -0.413058, 0.318804]  )# 17
quan_test_xyzw.append(           [0.255517, 0.833625, -0.153565, 0.464971]  )# 17
postition_xyz_for_euler.append(   [-482.13,-471.06,290.33] )# 17
euler_xyz.append(              [0.494,2.071,-0.221]    )# 17



postition_xyz_for_quan.append(   [-0.534567, -0.462644, 0.339583]  )# 18
quan_test_xyzw.append(         [0.22573, 0.850388, -0.0567135, 0.471878]    )# 18
postition_xyz_for_euler.append(   [-478.79,-477.79,314.45] )# 18
euler_xyz.append(           [0.491, 2.094, -0.115]       )# 18





postition_xyz_for_quan.append(   [-0.486852, -0.404617, 0.347691]  )# 19
quan_test_xyzw.append(            [0.313918, 0.837323, -0.0893024, 0.438601] )# 19
postition_xyz_for_euler.append(   [-463.04,-467.4,311.94] )# 19
euler_xyz.append(                [0.647,2.162,-0.217]  )# 19


postition_xyz_for_quan.append(   [-0.486852, -0.404617, 0.347691]  )# 20
quan_test_xyzw.append(           [0.313918, 0.837323, -0.0893024, 0.438601]  )# 20
postition_xyz_for_euler.append(  [-489.34,-474.61,288.34]  )# 20
euler_xyz.append(                 [0.582,2.011,-0.366] )# 20






postition_xyz_for_quan.append(   [-0.350432, -0.551071, 0.290331]  )# 21
quan_test_xyzw.append(            [-0.358644, -0.790081, -0.157994, 0.471364] )# 21
postition_xyz_for_euler.append(   [-454.04,-507.32,276.73] )# 21
euler_xyz.append(                 [1.88,3.106,0.465] )# 21





postition_xyz_for_quan.append( [-0.402958, -0.535988, 0.326388]    )# 22
quan_test_xyzw.append(         [-0.376768, -0.810052, -0.0872318, 0.440741]    )# 22
postition_xyz_for_euler.append(  [-454.81,-506.21,301.50]  )# 22
euler_xyz.append(               [1.572,3.127,0.179]   )# 22





postition_xyz_for_quan.append(   [-0.426661, -0.518751, 0.287169]  )# 23
quan_test_xyzw.append(           [-0.417105, -0.795657, -0.114605, 0.424051]  )# 23
postition_xyz_for_euler.append(  [-476.99,-508.09,264.61]  )# 23
euler_xyz.append(                 [1.094,3.241,0.211] )# 23





postition_xyz_for_quan.append(  [-0.407146, -0.535747, 0.327343]   )# 24
quan_test_xyzw.append(         [-0.372002, -0.815185, -0.153675, 0.416499]    )# 24
postition_xyz_for_euler.append(   [-453.81,-498.49,300.25] )# 24
euler_xyz.append(              [1.675,3.069,0.296]    )# 24





postition_xyz_for_quan.append(   [-0.405548, -0.544401, 0.308686]  )# 25
quan_test_xyzw.append(           [-0.464463, -0.77389, -0.160873, 0.399359]  )# 25
postition_xyz_for_euler.append(  [-453.5,-508.24,279.14]  )# 25
euler_xyz.append(                [2.112,2.797,0.278]  )# 25





postition_xyz_for_quan.append(   [-0.494603, -0.533271, 0.604086]  )# 26
quan_test_xyzw.append(           [-0.483111, -0.873634, 0.0165876, 0.055616]  )# 26
postition_xyz_for_euler.append(  [-496.27,-541.34,544.57]  )# 26
euler_xyz.append(               [1.475,2.605,-0.091]   )# 26





postition_xyz_for_quan.append(   [-0.464222, -0.527406, 0.614639]  )# 27
quan_test_xyzw.append(           [-0.448803, -0.887678, -0.0905463, 0.0490475]  )# 27
postition_xyz_for_euler.append(  [-457.52,-526.22,560.42]  )# 27
euler_xyz.append(               [1.41,2.604,-0.088]   )# 27





postition_xyz_for_quan.append(  [-0.457939, -0.518196, 0.592908]   )# 28
quan_test_xyzw.append(          [-0.425467, -0.899843, -0.0862857, 0.0426024]   )# 28
postition_xyz_for_euler.append(  [-479.81,-498.62,534.37]  )# 28
euler_xyz.append(                [1.391,2.567,-0.028]  )# 28





postition_xyz_for_quan.append(   [-0.486037, -0.523141, 0.560029]  )# 29
quan_test_xyzw.append(           [-0.49641, -0.863089, -0.0785204, 0.049893]  )# 29
postition_xyz_for_euler.append(  [-483.23,-497.5,504.61]  )# 29
euler_xyz.append(                [1.575,2.461,-0.022]  )# 29





postition_xyz_for_quan.append(  [-0.493251, -0.519436, 0.614022]   )# 30
quan_test_xyzw.append(          [-0.512539, -0.853589, -0.0183267, 0.0913967]   )# 30
postition_xyz_for_euler.append(  [-492.45,-513.55,557.13]  )# 30
euler_xyz.append(               [1.651,2.676,0.04]   )# 30





postition_xyz_for_quan.append(   [-0.555964, -0.48333, 0.287995]  )# 31
quan_test_xyzw.append(            [-0.418079, -0.895201, -0.121002, 0.095834] )# 31
postition_xyz_for_euler.append(   [-496.33,-487.46,236.33] )# 31
euler_xyz.append(             [1.26,2.546,0.237]     )# 31





postition_xyz_for_quan.append(  [-0.502614, -0.472583, 0.279817]   )# 32
quan_test_xyzw.append(          [-0.481809, -0.856238, -0.0888127, 0.163797]   )# 32
postition_xyz_for_euler.append( [-475.48,-497.16,231.35]   )# 32
euler_xyz.append(              [1.503,2.626,0.032]    )# 32





postition_xyz_for_quan.append(  [-0.466603, -0.496818, 0.280813]   )# 33
quan_test_xyzw.append(          [-0.429412, -0.87514, -0.14983, 0.165182]   )# 33
postition_xyz_for_euler.append(  [-453.06,-500.14,222.04]  )# 33
euler_xyz.append(               [1.431,2.664,0.246]   )# 33





postition_xyz_for_quan.append(  [-0.491281, -0.494506, 0.27947]   )# 34
quan_test_xyzw.append(           [-0.410026, -0.897458, -0.0945901, 0.132293]  )# 34
postition_xyz_for_euler.append(  [-469.76,-491.09,229.88]  )# 34
euler_xyz.append(              [1.319,2.691,0.061]    )# 34





postition_xyz_for_quan.append(  [-0.490642, -0.465303, 0.293394]   )# 35
quan_test_xyzw.append(          [-0.439376, -0.891693, -0.0249605, 0.105874]   )# 35
postition_xyz_for_euler.append( [-474.74,-498.6,228.08]   )# 35
euler_xyz.append(             [1.361,2.752,0.04]     )# 35

# # # # postition_xyz_for_euler_post =[[]]
# # # # euler_xyz_post = [[]]


# # # # postition_xyz_for_quan.append(   [-0.458623, -0.497324, 0.309447])# 36
# # # # quan_test_xyzw.append(          [-0.432701, -0.895101, -0.0446944, 0.0978091]   )# 36
# # # # postition_xyz_for_euler_post.append(  [-450.89, 503.72, 312.09]  )# 36
# # # # euler_xyz_post.append(                [1.43, 3.008, 0.143]  )# 36
# # # # postition_xyz_for_euler.append(  [-465.15, -494.74, 257.39]  )# 36
# # # # euler_xyz.append(        [1.39, 2.692, 0.018]     )# 36





# # # # postition_xyz_for_quan.append(   [-0.402648, -0.533267, 0.302131]  )# 37
# # # # quan_test_xyzw.append(           [-0.429691, -0.884434, -0.0683743, 0.168723]  )# 37
# # # # postition_xyz_for_euler_post.append( [-394.44, -538.75, 304.74]   )# 37
# # # # euler_xyz_post.append(               [1.495, 3.13, 0.233]   )# 37
# # # # postition_xyz_for_euler.append(   [-433.17, -499.29, 260.65] )# 37
# # # # euler_xyz.append(           [1.51, 2.637, 0.033]  )# 37





# # # # postition_xyz_for_quan.append(   [-0.483612, -0.456196, 0.311442]  )# 38
# # # # quan_test_xyzw.append(           [0.296686, 0.857547, -0.102537, 0.407525]  )# 38
# # # # postition_xyz_for_euler_post.append(  [-476.19, -463.45, 314.64]  )# 38
# # # # euler_xyz_post.append(           [0.730, 2.163, -0.256]       )# 38
# # # # postition_xyz_for_euler.append(   [-467.94, -468.56, 271.52] )# 38
# # # # euler_xyz.append(            [0.666, 2.323, -0.368] )# 38





# # # # postition_xyz_for_quan.append(  [-0.509075, -0.579167, 0.268675]   )# 39
# # # # quan_test_xyzw.append(           [-0.497107, -0.708844, -0.499857, 0.0238305]  )# 39
# # # # postition_xyz_for_euler_post.append(  [-500.61, -585.77, 272.03]  )# 39
# # # # euler_xyz_post.append(                [1.565, 2.277, 1.588]  )# 39
# # # # postition_xyz_for_euler.append( [-472.2, -509.88, 255.48]   )# 39
# # # # euler_xyz.append(        [1.729, 2.046, 1.356]     )# 39





# # # # postition_xyz_for_quan.append(   [-0.391013, -0.524644, 0.286302]  )# 40
# # # # quan_test_xyzw.append(           [-0.385108, -0.790193, -0.152702, 0.45163]  )# 40
# # # # postition_xyz_for_euler_post.append( [-383.33, -529.62, 288.78]   )# 40
# # # # euler_xyz_post.append(            [1.737, 3.623, 0.68]      )# 40
# # # # postition_xyz_for_euler.append(  [-480, -510.93, 282.16]  )# 40
# # # # euler_xyz.append(           [1.851, 3.107, 0.295]  )# 40





# postition_xyz_for_quan.append(     )# 0
# quan_test_xyzw.append(             )# 0
# postition_xyz_for_euler.append(    )# 0
# euler_xyz_post.append(             )# 0




################################################################################################


class poseSender(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('pose_sender')
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.main)



    def main(self):
        frame_to_view=3
        theory_transfer_frames = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            # theory_transfer_frames = np.append(theory_transfer_frames,theory_transfer_frame(quan_test_xyzw[i],postition_xyz_for_quan[i]),axis=0)
            theory_transfer_frames.append(theory_transfer_frame(quan_test_xyzw[i],postition_xyz_for_quan[i]))
            if i ==frame_to_view:
                tf_frame13 = TransformStamped()

                tf_frame13.header.stamp = self.get_clock().now().to_msg()
                tf_frame13.header.frame_id = 'world'
                tf_frame13.child_frame_id = 'theory_transfer_frames'
                tf_frame13.transform.translation.x = theory_transfer_frames[i][0][3]
                tf_frame13.transform.translation.y = theory_transfer_frames[i][1][3]
                tf_frame13.transform.translation.z = theory_transfer_frames[i][2][3]

                tf_quan13 = tf.quaternion_from_matrix(theory_transfer_frames[i])


                tf_frame13.transform.rotation.x = tf_quan13[0]
                tf_frame13.transform.rotation.y = tf_quan13[1]
                tf_frame13.transform.rotation.z = tf_quan13[2]
                tf_frame13.transform.rotation.w = tf_quan13[3]

                self.tf_broadcaster.sendTransform(tf_frame13)


        physical_transfer_frames = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            print("iteration: ", i)
            print("euler: ", euler_xyz) 
            print("euler postition: ", postition_xyz_for_euler)
            # physical_transfer_frames = np.append(physical_transfer_frames, physical_transfer_frame(physical_hand_frame(euler_xyz[i],postition_xyz_for_euler[i])),axis=0)
            physical_transfer_frames.append(physical_transfer_frame(physical_hand_frame(euler_xyz[i],postition_xyz_for_euler[i])))
            

            if i ==frame_to_view:
                tf_frame13 = TransformStamped()

                tf_frame13.header.stamp = self.get_clock().now().to_msg()
                tf_frame13.header.frame_id = 'world'
                tf_frame13.child_frame_id = 'physical_transfer_frames'
                tf_frame13.transform.translation.x = physical_transfer_frames[i][0][3]
                tf_frame13.transform.translation.y = physical_transfer_frames[i][1][3]
                tf_frame13.transform.translation.z = physical_transfer_frames[i][2][3]

                # frame = [ [ -0.5455522,  0.7925707, -0.2724051,0],
                #           [   0.7563086,  0.6056271,  0.2474129,0],
                #           [   0.3610682, -0.0710456, -0.9298292,0 ],
                #           [0,0,0,1] ]

                tf_quan13 = tf.quaternion_from_matrix(physical_transfer_frames[i])
                # tf_quan13 = tf.quaternion_from_matrix(frame)


                tf_frame13.transform.rotation.x = tf_quan13[0]
                tf_frame13.transform.rotation.y = tf_quan13[1]
                tf_frame13.transform.rotation.z = tf_quan13[2]
                tf_frame13.transform.rotation.w = tf_quan13[3]

                self.tf_broadcaster.sendTransform(tf_frame13)

        error_distances = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break

            t = np.array([theory_transfer_frames[i][0][3], theory_transfer_frames[i][1][3], theory_transfer_frames[i][2][3]])

            p = np.array([physical_transfer_frames[i][0][3], physical_transfer_frames[i][1][3], physical_transfer_frames[i][2][3]])



            # error_distances = np.append(error_distances,np.linalg.norm(t-p),axis=0)

            error_distances.append(np.linalg.norm(t-p))

        error_angles = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            print("iteration: ",i)
            t = [theory_transfer_frames[i][0][2], theory_transfer_frames[i][1][2], theory_transfer_frames[i][2][2]]

            p = [physical_transfer_frames[i][0][2], physical_transfer_frames[i][1][2], physical_transfer_frames[i][2][2]]
            print("physical normal: ",p)
            print("theory normal: ",t)

            # p = [0.951541,-0.1116404,-0.9891825]
            # error_angles = np.append( error_angles ,  angle_between(t[i],p[i]),axis=0)
            error_angles.append(angle(t,p)* 180/np.pi)


        print("distance error: ", error_distances)
        print("angle error: ", error_angles)
        print("axis angle: ",)



    def main_post(self):
        frame_to_view=5
        theory_transfer_frames = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            # theory_transfer_frames = np.append(theory_transfer_frames,theory_transfer_frame(quan_test_xyzw[i],postition_xyz_for_quan[i]),axis=0)
            theory_transfer_frames.append(theory_transfer_frame(quan_test_xyzw[i],postition_xyz_for_quan[i]))
            if i ==frame_to_view:
                tf_frame13 = TransformStamped()

                tf_frame13.header.stamp = self.get_clock().now().to_msg()
                tf_frame13.header.frame_id = 'world'
                tf_frame13.child_frame_id = 'theory_transfer_frames'
                tf_frame13.transform.translation.x = theory_transfer_frames[i][0][3]
                tf_frame13.transform.translation.y = theory_transfer_frames[i][1][3]
                tf_frame13.transform.translation.z = theory_transfer_frames[i][2][3]

                tf_quan13 = tf.quaternion_from_matrix(theory_transfer_frames[i])


                tf_frame13.transform.rotation.x = tf_quan13[0]
                tf_frame13.transform.rotation.y = tf_quan13[1]
                tf_frame13.transform.rotation.z = tf_quan13[2]
                tf_frame13.transform.rotation.w = tf_quan13[3]

                self.tf_broadcaster.sendTransform(tf_frame13)


        physical_transfer_frames = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            # physical_transfer_frames = np.append(physical_transfer_frames, physical_transfer_frame(physical_hand_frame(euler_xyz[i],postition_xyz_for_euler[i])),axis=0)
            physical_transfer_frames.append(physical_hand_frame(euler_xyz_post[i],postition_xyz_for_euler_post[i]))
            
            if i ==frame_to_view:
                tf_frame13 = TransformStamped()

                tf_frame13.header.stamp = self.get_clock().now().to_msg()
                tf_frame13.header.frame_id = 'world'
                tf_frame13.child_frame_id = 'physical_transfer_frames'
                tf_frame13.transform.translation.x = physical_transfer_frames[i][0][3]
                tf_frame13.transform.translation.y = physical_transfer_frames[i][1][3]
                tf_frame13.transform.translation.z = physical_transfer_frames[i][2][3]

                # frame = [ [ -0.5455522,  0.7925707, -0.2724051,0],
                #           [   0.7563086,  0.6056271,  0.2474129,0],
                #           [   0.3610682, -0.0710456, -0.9298292,0 ],
                #           [0,0,0,1] ]

                tf_quan13 = tf.quaternion_from_matrix(physical_transfer_frames[i])
                # tf_quan13 = tf.quaternion_from_matrix(frame)


                tf_frame13.transform.rotation.x = tf_quan13[0]
                tf_frame13.transform.rotation.y = tf_quan13[1]
                tf_frame13.transform.rotation.z = tf_quan13[2]
                tf_frame13.transform.rotation.w = tf_quan13[3]

                self.tf_broadcaster.sendTransform(tf_frame13)

        error_distances = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break

            t = np.array([theory_transfer_frames[i][0][3], theory_transfer_frames[i][1][3], theory_transfer_frames[i][2][3]])

            p = np.array([physical_transfer_frames[i][0][3], physical_transfer_frames[i][1][3], physical_transfer_frames[i][2][3]])



            # error_distances = np.append(error_distances,np.linalg.norm(t-p),axis=0)

            error_distances.append(np.linalg.norm(t-p))

        error_angles = [[]]
        for i in range(len(quan_test_xyzw)):
            i +=1
            if i == len(quan_test_xyzw):
                break
            print("iteration: ",i)
            t = [theory_transfer_frames[i][0][2], theory_transfer_frames[i][1][2], theory_transfer_frames[i][2][2]]

            p = [physical_transfer_frames[i][0][2], physical_transfer_frames[i][1][2], physical_transfer_frames[i][2][2]]
            print("physical normal: ",p)
            print("theory normal: ",t)

            # p = [0.951541,-0.1116404,-0.9891825]
            # error_angles = np.append( error_angles ,  angle_between(t[i],p[i]),axis=0)
            error_angles.append(angle(t,p)* 180/np.pi)


        print("distance error: ", error_distances)
        print("angle error: ", error_angles)
        print("axis angle: ",)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    global poseSender
    # Create the node
    poseSender = poseSender()

    # Spin the node so the callback function is called.
    rclpy.spin(poseSender)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    poseSender.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    # rclpy.init(args=sys.argv)
    main()

