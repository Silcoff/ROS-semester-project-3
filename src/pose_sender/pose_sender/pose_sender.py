# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import tf_transformations as tf
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import traceback
import time


class poseSender(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        # initialize the node and give it a name (pose_sender).
        super().__init__('pose_sender')

        # create a tf broadcaster that will publish a TransformStamped message.
        self.tf_broadcaster = TransformBroadcaster(self)


        self.sub = self.create_subscription(Pose,"hand_pose_msg",self.poseCallback,1) # create a subscriber that will subscribe to a Pose message on the topic hand_pose_msg
        self.sub_timer = self.create_subscription(Bool,"timer",self.timerstop,1) # create a subscriber that will subscribe to a Bool message on the topic timer

        self.pub = self.create_publisher(Pose, 'pose_msg', 1) # create a publisher that will publish a Pose message on the topic pose_msg


    def timerstop(self,msg):
        try:
            print(msg.data)
            end_time = time.time()

            # calculate the time it took to move the robot
            timer =end_time-self.start_time        

            print("time to move: ", timer)
        except:
            traceback.print_exc()
            return


    
    def quaternion_rotation_matrix(self,Q):
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

    def poseCallback(self,data):

        in_quan_x = data.orientation.x
        in_quan_y = data.orientation.y
        in_quan_z = data.orientation.z
        in_quan_w = data.orientation.w

        in_quan = [in_quan_w,in_quan_x,in_quan_y,in_quan_z]

        in_post_x = data.position.x
        in_post_y = data.position.y
        in_post_z = data.position.z

        in_rot_matrix = self.quaternion_rotation_matrix(in_quan)
        

        

        ####################################


        cam_x = -505.94
        cam_y = -512.85
        cam_z = -58.24

        cam_RX = -3.137
        cam_RY = 0.001
        cam_RZ = -2.340









        frame_base_camera = [[np.cos(45*np.pi /180), -np.sin(45*np.pi /180),  0.000000 , cam_x/1000],
                       [np.sin(45*np.pi /180),  np.cos(45*np.pi /180),  0.000000 , cam_y/1000],
                       [0                    , 0                     , 1         , cam_z/1000],
                       [0                    , 0                     , 0         , 1     ]]



        




        ####### send tf frame_base_camera   #########################################
        tf_frame_base_camera = TransformStamped()

        tf_frame_base_camera.header.stamp = self.get_clock().now().to_msg()
        tf_frame_base_camera.header.frame_id = 'base'
        tf_frame_base_camera.child_frame_id = 'frame_base_camera'
        tf_frame_base_camera.transform.translation.x = frame_base_camera[0][3]
        tf_frame_base_camera.transform.translation.y = frame_base_camera[1][3]
        tf_frame_base_camera.transform.translation.z = frame_base_camera[2][3]

        tf_quan_base_camera = tf.quaternion_from_matrix(frame_base_camera)


        tf_frame_base_camera.transform.rotation.x = tf_quan_base_camera[0]
        tf_frame_base_camera.transform.rotation.y = tf_quan_base_camera[1]
        tf_frame_base_camera.transform.rotation.z = tf_quan_base_camera[2]
        tf_frame_base_camera.transform.rotation.w = tf_quan_base_camera[3]

        self.tf_broadcaster.sendTransform(tf_frame_base_camera)
        ################ tf send for frame_base_camera #############################




        frame_camera_hand =  [[in_rot_matrix[0][0], in_rot_matrix[0][1], in_rot_matrix[0][2], in_post_x],
                              [in_rot_matrix[1][0], in_rot_matrix[1][1], in_rot_matrix[1][2], in_post_y],
                              [in_rot_matrix[2][0], in_rot_matrix[2][1], in_rot_matrix[2][2], in_post_z],
                              [0                  , 0                  , 0                  , 1        ]]


        dgree90 = [[-1 , 0   , 0 , 0],
                   [0  , -1  , 0 , 0],
                   [0  , 0   , 1 , 0],
                   [0  , 0   , 0 , 1]]

        frame_camera_hand = np.dot(dgree90,frame_camera_hand)

        frame_base_camera = np.dot(frame_base_camera,frame_camera_hand)
        
        
        ####### send tf frame_base_camera   #########################################
        tf_frame_base_camera = TransformStamped()

        tf_frame_base_camera.header.stamp = self.get_clock().now().to_msg()
        tf_frame_base_camera.header.frame_id = 'base'
        tf_frame_base_camera.child_frame_id = 'frame_base_hand'
        tf_frame_base_camera.transform.translation.x = frame_base_camera[0][3]
        tf_frame_base_camera.transform.translation.y = frame_base_camera[1][3]
        tf_frame_base_camera.transform.translation.z = frame_base_camera[2][3]

        tf_quan_base_camera = tf.quaternion_from_matrix(frame_base_camera)


        tf_frame_base_camera.transform.rotation.x = tf_quan_base_camera[0]
        tf_frame_base_camera.transform.rotation.y = tf_quan_base_camera[1]
        tf_frame_base_camera.transform.rotation.z = tf_quan_base_camera[2]
        tf_frame_base_camera.transform.rotation.w = tf_quan_base_camera[3]

        self.tf_broadcaster.sendTransform(tf_frame_base_camera)
        ################ tf send for frame_base_camera #############################




        frame_hand_transfer  = np.array([[-1.0,   0.0,   0.0,  0.0], 
                                         [ 0.0,  -1.0,   0.0,  0.0],
                                         [ 0.0,   0.0,  -1.0,  0.1],
                                         [ 0.0,   0.0,   0.0,  1.0]])

        frame_base_transfer = np.dot(frame_base_camera, frame_hand_transfer)


        ####### send tf frame_base_transfer   #########################################
        tf_frame_base_transfer = TransformStamped()

        tf_frame_base_transfer.header.stamp = self.get_clock().now().to_msg()
        tf_frame_base_transfer.header.frame_id = 'base'
        tf_frame_base_transfer.child_frame_id = 'frame_base_transfer'
        tf_frame_base_transfer.transform.translation.x = frame_base_transfer[0][3]
        tf_frame_base_transfer.transform.translation.y = frame_base_transfer[1][3]
        tf_frame_base_transfer.transform.translation.z = frame_base_transfer[2][3]

        tf_quan_base_transfer = tf.quaternion_from_matrix(frame_base_transfer)


        tf_frame_base_transfer.transform.rotation.x = tf_quan_base_transfer[0]
        tf_frame_base_transfer.transform.rotation.y = tf_quan_base_transfer[1]
        tf_frame_base_transfer.transform.rotation.z = tf_quan_base_transfer[2]
        tf_frame_base_transfer.transform.rotation.w = tf_quan_base_transfer[3]

        self.tf_broadcaster.sendTransform(tf_frame_base_transfer)
        ################ tf send for frame_base_transfer #############################





         
        out_quan = tf.quaternion_from_matrix(frame_base_transfer)



        hand_pose = Pose()

        hand_pose.orientation.x = out_quan[0]
        hand_pose.orientation.y = out_quan[1]
        hand_pose.orientation.z = out_quan[2]
        hand_pose.orientation.w = out_quan[3]

        hand_pose.position.x = float(frame_base_transfer[0][3])
        hand_pose.position.y = float(frame_base_transfer[1][3])
        hand_pose.position.z = float(frame_base_transfer[2][3])


        self.pub.publish(hand_pose)
        self.start_time = time.time()



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

