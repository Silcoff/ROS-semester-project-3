# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Vector3
import numpy as np

class poseSender(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('pose_sender')
        self.sub = self.create_subscription(Vector3,"hand_pose_msg",self.poseCallback,1)

        self.pub = self.create_publisher(Vector3, 'pose_msg', 1)
        # self.poseCallback()
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.poseCallback)


    def frame3_to_frame1(self, ai, aj, ak):

        frame12 = np.array([    [0.7071068, -0.7071068,  0.0000000, 0.0000000], 
                                [0.7071068,  0.7071068,  0.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  1.0000000, 0.0000000],
                                [0.0000000,  0.0000000,  0.0000000, 1.0000000]])

        frame23 = np.array([    [1.0,  0.0,  0.0,    1.0], 
                                [0.0,  1.0,  0.0,    0.0],
                                [0.0,  0.0,  1.0,   -0.1],
                                [0.0,  0.0,  0.0,    1.0]])

        frame13=np.dot(frame12,frame23)

        frame31=np.linalg.inv(frame13)


        P = np.array([ai,aj,ak,1])
        v = np.dot(frame13,P)
        q=v[0:3]
        return q


    def poseCallback(self,data):
        self.x= float(data.x)/1000
        self.y= float(data.y)/1000
        self.z= float(data.z)/1000
        print(data.x)
        print(data.y)
        print(data.z)




        # self.x = float(input("x: "))
        # self.y = float(input("y: "))
        # self.z = float(input("z: "))



        self.frame31=self.frame3_to_frame1(self.x,self.y,self.z)



        self.pose_msg = Vector3()
        

        # self.result=self.base_to_frame1(self.x,self.y,self.z)

        self.pose_msg.x = float(self.frame31[0])
        self.pose_msg.y = float(self.frame31[1])
        self.pose_msg.z = float(self.frame31[2])
        


        print(self.pose_msg)
        self.pub.publish(self.pose_msg)


        # while(True):
        #     self.pose_msg = Vector3()
            
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