# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from moveit_msgs.msg import CartesianPoint
from geometry_msgs.msg import Vector3
import numpy as np


class poseSender(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('pose_sender')

        self.pub = self.create_publisher(Vector3, 'pose_msg', 1)
        self.poseCallback()
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.poseCallback)


    def quaternion_from_euler(self, ai, aj, ak):
        # ai /= 2.0
        # aj /= 2.0
        # ak /= 2.0
        # ci = math.cos(ai)
        # si = math.sin(ai)
        # cj = math.cos(aj)
        # sj = math.sin(aj)
        # ck = math.cos(ak)
        # sk = math.sin(ak)
        # cc = ci*ck
        # cs = ci*sk
        # sc = si*ck
        # ss = si*sk

        # q = np.empty((4, ))
        # q[0] = cj*sc - sj*cs
        # q[1] = cj*ss + sj*cc
        # q[2] = cj*cs - sj*sc
        # q[3] = cj*cc + sj*ss

        R = np.array([  [0.7071068, -0.7071068,  0.0000000], [  0.7071068,  0.7071068,  0.0000000],[0.0000000,  0.0000000,  1.0000000 ]])

        P = np.array([ai,aj,ak])
        q = [0,0,0]

        # print(R*P)
        q[0] = R[0][0] * ai + R[0][1] * aj + R[0][2] * ak
        q[1] = R[1][0] * ai + R[1][1] * aj + R[1][2] * ak
        q[2] = R[2][0] * ai + R[2][1] * aj + R[2][2] * ak
        


        return q

    def poseCallback(self):

        while(True):
            self.pose_msg = Vector3()
            
            self.x = float(input("x: "))
            self.y = float(input("y: "))
            self.z = float(input("z: "))

            self.result=self.quaternion_from_euler(self.x,self.y,self.z)

            self.pose_msg.x = self.result[0]
            self.pose_msg.y = self.result[1]
            self.pose_msg.z = self.result[2]
            
            print(self.pose_msg)
            self.pub.publish(self.pose_msg)





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