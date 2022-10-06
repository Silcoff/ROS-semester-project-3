# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image as msg_Image # Image is the message type

from cv_bridge import CvBridge
import cv2 as cv

class ImageListener(Node):
    # this funciton will run when the class is called upon
    def __init__(self, color_image_topic):
        super().__init__('show_center_depth')
        # we instantiate the cvbridge oject
        self.bridge = CvBridge()

        # we create an subcriber where the first input is the type of msg, second is the topic that we subscribe to, third is the callbackfunction the data will be passed to, the last element i do not know what is.
        self.sub = self.create_subscription(msg_Image, color_image_topic, self.imageColorCallback, 1)

        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is unkown
        self.pub = self.create_publisher(msg_Image, 'video_frames', 1)



    def imageColorCallback(self, data):
            # here the data passed in is converted into a cv2 array with a heigt and width of the realsense camera with a pixel like BGR
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            # here we take the bgr image and use opencv to turn it into rgb
            self.rgb_cv_image =cv.cvtColor(self.cv_image, cv.COLOR_BGR2RGB)

            # the converted image we then convert into a ROS msg and the publish it
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.rgb_cv_image))




def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)


    #sets the topics to subscribe to
    color_image_topic = '/camera/color/image_raw'


    # Create the node
    listener = ImageListener(color_image_topic)

    # Spin the node so the callback function is called.
    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    # rclpy.init(args=sys.argv)
    main()
