import pyrealsense2 as rs
import numpy as np

# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image as msg_Image # Image is the message type
from geometry_msgs.msg import Vector3

from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2 as cv



from turtle import width
import numpy as np
# from realsense_depth import *
import math



class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()




class ImageListener(Node):
    # this funciton will run when the class is called upon
    def __init__(self, color_image_topic):
        super().__init__('test_code')
        # we instantiate the cvbridge oject
        self.bridge = CvBridge()

        self.searchBool = True

        self.upperThresh = 70
        self.lowerThresh = 40
        self.dc = DepthCamera()
        self.Height  = 300
        self.Width   = 500
        self.FOV = [69,42] 
        self.margin = 60


        # we create an subcriber where the first input is the type of msg, second is the topic that we subscribe to, third is the callbackfunction the data will be passed to, the last element i do not know what is.
        # self.sub = self.create_subscription(msg_Image, color_image_topic, self.find_hand_callback, 1)

        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is unkown
        self.pub = self.create_publisher(Vector3, 'hand_pose_msg', 1)
        self.create_subscription(Bool, 'searchBool',self.change_Bool_callback,0)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.find_hand_callback)



    def change_Bool_callback(self,data):

        self.searchBool = data

    def find_hand_callback(self):

        if self.searchBool == False:
            return


        ret, depth_frame, img = self.dc.get_frame()
        
        dCrop = depth_frame[self.margin:depth_frame.shape[0]-self.margin, self.margin:depth_frame.shape[1]-self.margin]

        img = img[self.margin:img.shape[0]-self.margin, self.margin:img.shape[1]-self.margin]
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #converts an image to grayscale
        gray = cv.GaussianBlur(gray, (3,3), cv.BORDER_DEFAULT)

        canny = cv.Canny(gray, self.lowerThresh, self.upperThresh)

        kSize = 100
        kernel = np.ones((kSize, kSize), np.uint8)

        closeThresh = cv.erode(cv.dilate(canny,kernel),kernel)


        distTrans= cv.distanceTransform(closeThresh, cv.DIST_L2, 3)

        cv.normalize(distTrans, distTrans, 0, 1.0, cv.NORM_MINMAX)

        final = cv.cvtColor(distTrans, cv.COLOR_GRAY2BGR)

        maxX = 0
        maxY = 0
        maxVal = 0
        for y in range(self.Height):
            for x in range(self.Width):
                if distTrans[y,x] != 0 and distTrans[y,x] > maxVal:
                    maxVal = distTrans[y,x]
                    maxX = x
                    maxY = y
        
        angX = (maxX * (self.FOV[0]/self.Width)) - self.FOV[0]/2
        angY = (maxY * (self.FOV[1]/self.Height)) - self.FOV[1]/2
        d = dCrop[maxY,maxX]

        xPos = math.sin(math.radians(angX))*d
        yPos = math.sin(math.radians(angY))*d
        zPos = d


        self.pose_msg = Vector3()
        

        self.pose_msg.x = float(xPos)
        self.pose_msg.y = float(yPos)
        self.pose_msg.z = float(zPos)
        # the converted image we then convert into a ROS msg and the publish it
        self.pub.publish(self.pose_msg)

        self.searchBool = False




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
