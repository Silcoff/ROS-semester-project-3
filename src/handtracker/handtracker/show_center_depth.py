# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image as msg_Image # Image is the message type

from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

import math

from std_msgs.msg import Bool


class ImageListener(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('test_code')
        # we instantiate the cvbridge oject
        self.bridge = CvBridge()
        
        # we create an subcriber where the first input is the type of msg, second is the topic that we subscribe to, third is the callbackfunction the data will be passed to, the last element i do not know what is.
        self.sub_bgr = self.create_subscription(msg_Image, "/camera/color/image_raw", self.imageColorCallback, 1)
        self.sub_depth = self.create_subscription(msg_Image, "/camera/depth/image_rect_raw", self.depthColorCallback, 2)
        self.pub_bgr = self.create_publisher(msg_Image,"/bgr_image",1)
        self.pub_depth = self.create_publisher(msg_Image,"/depth_image",1)

        
        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is unkown
        self.pub_pose = self.create_publisher(Vector3, 'hand_pose_msg', 1)
        self.create_subscription(Bool, 'searchBool',self.change_Bool_callback,0)
        # timer_period = 5  # seconds
        # self.timer = self.create_timer(timer_period, self.depthColorCallback)

        self.searchBool = True

        self.bgr_image = None
        self.depth_image = None

        self.FOV = [69,42] 



    def change_Bool_callback(self,data):
        self.searchBool = data


    def imageColorCallback(self, data):
        self.bgr_image = np.array(self.bridge.imgmsg_to_cv2(data, data.encoding))
        # self.sub_depth = self.create_subscription(msg_Image, "/camera/depth/image_rect_raw", self.depthColorCallback, 2)

        

    def depthColorCallback(self, data):


        if self.searchBool == False:
            return

        self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        # threshold depth image
        ret, self.tresh_depth_lowerbound = np.array(cv.threshold(self.depth_image,200,500,cv.THRESH_TOZERO))
        ret, self.tresh_depth_upperbound = np.array(cv.threshold(self.tresh_depth_lowerbound,650,500,cv.THRESH_TOZERO_INV))

        # convert to binary
        ret, self.depth_binary = np.array(cv.threshold(self.tresh_depth_upperbound,0,500,cv.THRESH_BINARY))

        # ersion and dialaiton
        self.depth_dialted = self.dilatation(20,0,self.depth_binary)
        self.depth_eroded = self.erosion(40,2,self.depth_dialted)
        self.depth_dialted_final = np.array(self.dilatation(20,2,self.depth_eroded))

        # convert dialated depth to uint8
        self.depth_dialted_final = self.depth_dialted_final.astype('uint8')


        # compute distance transform on dialated depth
        self.distTrans= cv.distanceTransform(self.depth_dialted_final, cv.DIST_L2, 3)

        # find max values in dialated depth
        self.max_Value_Depth = np.amax(self.distTrans)

        # compute all places where max value in dialeted depth is
        self.XY_max_Depth_Loc = np.where(self.distTrans==self.max_Value_Depth)

        # get fisrt max value coordinate from dialated depth
        self.isolate_XY_max_Depth_Loc = [self.XY_max_Depth_Loc[1][0],self.XY_max_Depth_Loc[0][0]]

        # create circle around max value in dialated depth
        cv.circle(self.distTrans,self.isolate_XY_max_Depth_Loc,20,255,thickness=3)


        # find depth computed from the max value coordinates 
        self.hand_depth = self.depth_image[self.isolate_XY_max_Depth_Loc[1]][self.isolate_XY_max_Depth_Loc[0]]
        print(self.hand_depth)

        
        # publish distance-transform computed from dialated depth 
        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(self.distTrans ))

        maxY=self.isolate_XY_max_Depth_Loc[0]
        maxX=self.isolate_XY_max_Depth_Loc[1]

        self.Height = self.depth_image.shape[0]
        self.Width = self.depth_image.shape[1]

        angX = (maxX * (self.FOV[1]/self.Height)) - self.FOV[1]/2
        angY = (maxY * (self.FOV[0]/self.Width)) - self.FOV[0]/2

        xPos = math.sin(math.radians(angX))*self.hand_depth
        yPos = math.sin(math.radians(angY))*self.hand_depth
        zPos = self.hand_depth

        self.pose_msg = Vector3()
        

        self.pose_msg.x = -1*float(yPos)
        self.pose_msg.y = -1*float(xPos)
        self.pose_msg.z = float(zPos)


        # convert bgr image into rgb
        self.rgb_image = cv.cvtColor(self.bgr_image,cv.COLOR_BGR2RGB)

        # pubblish rgb image
        self.pub_bgr.publish(self.bridge.cv2_to_imgmsg(self.rgb_image))

        self.pub_pose.publish(self.pose_msg)

        # self.searchBool = False



    def morph_shape(self,val):
        if val == 0:
            return cv.MORPH_RECT
        elif val == 1:
            return cv.MORPH_CROSS
        elif val == 2:
            return cv.MORPH_ELLIPSE

    def erosion(self,erosion_size,shape,image):
        erosion_shape = self.morph_shape(shape)

        element = cv.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                        (erosion_size, erosion_size))

        erosion_dst = cv.erode(image, element)
        return erosion_dst

    def dilatation(self,dilatation_size,shape,image):

        dilation_shape = self.morph_shape(shape)
        
        element = cv.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),(dilatation_size, dilatation_size))
        
        dilatation_dst = cv.dilate(image, element)
        return dilatation_dst



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)


    # Create the node
    listener = ImageListener()

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
