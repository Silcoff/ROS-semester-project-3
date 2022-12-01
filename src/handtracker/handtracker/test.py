# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image as msg_Image # Image is the message type

from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

import math
import time

from sensor_msgs.msg import PointCloud2 

from std_msgs.msg import Bool


import sensor_msgs


class ImageListener(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('test_code')
        # we instantiate the cvbridge oject
        self.bridge = CvBridge()
        
        self.searchBool = True
        
        # we create an subcriber where the first input is the type of msg, second is the topic that we subscribe to, third is the callbackfunction the data will be passed to, the last element i do not know what is.
        # self.sub_bgr = self.create_subscription(msg_Image, "/camera/color/image_raw", self.colorImageCallback, 1)
        # self.sub_depth = self.create_subscription(msg_Image, "/camera/depth/image_rect_raw", self.depthImageCallback, 1)
        self.sub_depth = self.create_subscription(PointCloud2, "/depth/color/points", self.pointcloudCallback, 1)
        self.pub_bgr = self.create_publisher(msg_Image,"/rgb_image",1)
        self.pub_depth = self.create_publisher(msg_Image,"/depth_image",1)

        
        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is unkown
        self.pub_pose = self.create_publisher(Vector3, 'hand_pose_msg', 1)
        self.create_subscription(Bool, 'searchBool',self.change_Bool_callback,1)



        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.handtrackerCallback)

        self.rgb_image = None
        self.depth_image = None
        self.stupid=True
        self.FOV = [69,42] 



    def change_Bool_callback(self,data):
        self.searchBool = data.data


        print(self.searchBool)

    def thresholdDepth(self):
        # threshold depth image
        ret, self.tresh_depth_lowerbound = np.array(cv.threshold(self.depth_image,200,500,cv.THRESH_TOZERO))
        ret, self.tresh_depth_upperbound = np.array(cv.threshold(self.tresh_depth_lowerbound,650,500,cv.THRESH_TOZERO_INV))

        # convert to binary
        ret, self.depth_binary = np.array(cv.threshold(self.tresh_depth_upperbound,0,500,cv.THRESH_BINARY))


    def resizeDepthBGR(self):
        depthWidth = self.depth_image.shape[1]
        depthHeigth = self.depth_image.shape[0]
        rgbWidth  = self.rgb_image.shape[1]
        rgbHeigth = self.rgb_image.shape[0]

        widthDiff = int(abs(depthWidth-rgbWidth)/2)

  
        self.cropped_depth_image = self.depth_image[:,104:depthWidth-104]

        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(self.cropped_depth_image ))
        self.pub_bgr.publish(self.bridge.cv2_to_imgmsg(self.rgb_image ))




    def pointcloudCallback(self,data):
        
        arr = sensor_msgs.point_cloud2.read_points(data)
        print(arr)

        # cloud_msg = PointCloud2()

        # cloud_msg.is_dense(data)

        # if self.stupid==True:
        #     print(data)

        #     self.stupid=False




    def handtrackerCallback(self):
        # print("handtracker")
        if self.searchBool == False:
            print(self.searchBool)
            return
        try:
            self.thresholdDepth()


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

            
            # # publish distance-transform computed from dialated depth 
            # self.pub_depth.publish(self.bridge.cv2_to_imgmsg(self.distTrans ))

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

            if (self.pose_msg.z==0):
                return

            # convert bgr image into rgb
            self.rgb_image = cv.cvtColor(self.rgb_image,cv.COLOR_BGR2RGB)

            # # pubblish rgb image
            # self.pub_bgr.publish(self.bridge.cv2_to_imgmsg(self.rgb_image))

            self.pub_pose.publish(self.pose_msg)
            self.resizeDepthBGR()

            self.searchBool = False
        except:
            return




    def colorImageCallback(self, data):
        # print("image")
        # if self.searchBool == False:
        #     # print(self.searchBool)
        #     return        
        self.rgb_image = np.array(self.bridge.imgmsg_to_cv2(data, data.encoding))

        

    def depthImageCallback(self, data):
        # print("depth")
        # if self.searchBool == False:
        #     # print(self.searchBool)
        #     return
        self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)




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