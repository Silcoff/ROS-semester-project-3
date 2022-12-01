# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose
import numpy as np


from geometry_msgs.msg import Pose

import tf_transformations as tf



class poseSender(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('pose_sender')
        self.sub = self.create_subscription(Pose,"hand_pose_msg",self.poseCallback,1)

        self.pub = self.create_publisher(Pose, 'pose_msg', 1)
        # self.poseCallback()
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.poseCallback)


    def frame3_to_frame1(self, ai, aj, ak):

        frame12 = np.array([    [0.7071068, -0.7071068,  0.0000000, 0.0000000], 
                                [0.7071068,  0.7071068,  0.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  1.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  0.0000000, 1.0000000]])

        frame23 = np.array([    [1.0,  0.0,  0.0,  1.0], 
                                [0.0,  1.0,  0.0,  0.0],
                                [0.0,  0.0,  1.0,  0.05],
                                [0.0,  0.0,  0.0,  1.0]])

        frame13=np.dot(frame12,frame23)


        P = np.array([ai,aj,ak,1])
        v = np.dot(frame13,P)
        q=v[0:3]
        return frame13



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
        # self.x= float(data.x)/1000
        # self.y= float(data.y)/1000
        # self.z= float(data.z)/1000
        # print(data.x)
        # print(data.y)
        # print(data.z)


        in_quan_x = data.orientation.x
        in_quan_y = data.orientation.y
        in_quan_z = data.orientation.z
        in_quan_w = data.orientation.w

        in_quan = [in_quan_w,in_quan_x,in_quan_y,in_quan_z]

        in_post_x = data.position.x
        in_post_y = data.position.y
        in_post_z = data.position.z

        in_rot_matrix = self.quaternion_rotation_matrix(in_quan)
        

        frame34 =  [[in_rot_matrix[0][0], in_rot_matrix[0][1], in_rot_matrix[0][2], in_post_x],
                    [in_rot_matrix[1][0], in_rot_matrix[1][1], in_rot_matrix[1][2], in_post_y],
                    [in_rot_matrix[2][0], in_rot_matrix[2][1], in_rot_matrix[2][2], in_post_z],
                    [0                  , 0                  , 0                  , 1        ]]



        # self.x = float(input("x: "))
        # self.y = float(input("y: "))
        # self.z = float(input("z: "))



        # frame31=self.frame3_to_frame1(self.x,self.y,self.z)


        frame12 = np.array([    [0.7071068, -0.7071068,  0.0000000, 0.0000000], 
                                [0.7071068,  0.7071068,  0.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  1.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  0.0000000, 1.0000000]])

        frame23 = np.array([    [1.0,  0.0,  0.0,  1.0], 
                                [0.0,  1.0,  0.0,  0.0],
                                [0.0,  0.0,  1.0,  0.05],
                                [0.0,  0.0,  0.0,  1.0]])

        frame13=np.dot(frame12,frame23)

        frame14 = np.dot(frame13,frame34)

        reversed = np.array([   [-1.0,   0.0,   0.0,  0.0], 
                                [ 0.0,  -1.0,   0.0,  0.0],
                                [ 0.0,   0.0,  -1.0,  0.0],
                                [ 0.0,   0.0,   0.0,  1.0]])

        reversed_frame14 = np.dot(reversed,frame14)


        offset = np.array([     [ 1.0,   0.0,   0.0,  0.0], 
                                [ 0.0,   1.0,   0.0,  0.0],
                                [ 0.0,   0.0,   1.0,  0.1],
                                [ 0.0,   0.0,   0.0,  1.0]])
        offset_frame14 = np.dot(offset,frame14)


        out_quan = tf.quaternion_from_matrix(reversed_frame14)



        hand_pose = Pose()

        hand_pose.orientation.x = out_quan[0]
        hand_pose.orientation.y = out_quan[1]
        hand_pose.orientation.z = out_quan[2]
        hand_pose.orientation.w = out_quan[3]


        print(float(frame14[0][3]))
        print(float(frame14[1][3]))
        print(float(frame14[2][3]))
        hand_pose.position.x = float(frame14[0][3])
        hand_pose.position.y = float(frame14[1][3])
        hand_pose.position.z = float(frame14[2][3])





        self.pub.publish(hand_pose)


        # while(True):
        #     self.pose_msg = Pose()
            
        #     self.x = float(input("x: "))
        #     self.y = float(input("y: "))
        #     self.z = float(input("z: "))

        #     self.result=self.base_to_frame1(self.x,self.y,self.z)

        #     self.pose_msg.x = self.result[0]
        #     self.pose_msg.y = self.result[1]
        #     self.pose_msg.z = self.result[2]
            
        #     print(self.pose_msg)
        #     self.pub.publish(self.pose_msg)





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



# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1 


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()