# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
import time
import traceback
from matplotlib import pyplot as plt
from std_msgs.msg import Bool
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from math import sqrt
import traceback
from geometry_msgs.msg import Pose
import tf_transformations as tf




class ImageListener(Node):
    # this funciton will run when the class is called upon
    def __init__(self):
        super().__init__('Hand_detection')

        # the searchBool is a bool that is used to tell the handtracker if it should search for the hand or not
        self.searchBool = True



    
        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is the queue size.
        self.pub_pose = self.create_publisher(Pose, 'hand_pose_msg', 1)

        # we create a subscriber, where the first input is the msg type, second is the topic name, third is the callback function, fourth is the queue size.
        self.create_subscription(Bool, 'searchBool',self.change_Bool_callback,1)

        self.old_positions = np.zeros((5,3))


        timer_period = 0.2  # seconds
        # Create a timer, that runs a callback function every timer_period seconds.
        self.timer = self.create_timer(timer_period, self.handtracker_callback)

        # defiens the iteration startpoint ouside the loop
        self.iteration = 0

        # defines the start time outside the loop
        self.when_to_start_time = 0

        #####################################
        #####     test blob size   ##########
        #####################################

        self.blob_depth = []
        self.blob_size1 = []
        self.getdata = 0

        ##########################################################################
        ######     setup the realsense camera and the self.pipeline   ############
        ##########################################################################

        
        self.pipeline = rs.pipeline() # Create a self.pipeline object. This object configures the streaming camera and owns it's handle

        # Create a config object and configure the self.pipeline to stream different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline) 
        pipeline_profile = config.resolve(pipeline_wrapper) 
        device = pipeline_profile.get_device()

        # Check if the realsense camera has a RGB sensor
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # Configure streams 
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming with requested config
        profile = self.pipeline.start(config)

        profile_depth = profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
        self.camera_intrinsics  = profile_depth.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics


        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 0.65 # the distance is in meters
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames.
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color 
        self.align = rs.align(align_to)


    # this function will change the searchBool to true when a message is received.
    def change_Bool_callback(self,data):
        self.searchBool = data.data
        print(self.searchBool)


    # this function defines the shape of the morphological operations.
    def morph_shape(self,val):
        if val == 0:
            return cv.MORPH_RECT
        elif val == 2:
            return cv.MORPH_ELLIPSE


    # this funciton takes the input image and returns the image with the morphological operations applied.
    # erosion_size is the size of the erosion, shape is the shape of the erosion, image is the input image.
    def erosion(self,erosion_size,shape,image):
        erosion_shape = self.morph_shape(shape)

        element = cv.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                        (erosion_size, erosion_size))

        erosion_dst = cv.erode(image, element)
        return erosion_dst

    # this funciton takes the input image and returns the image with the morphological operations applied.
    # dilatation_size is the size of the dilatation, shape is the shape of the dilatation, image is the input image.
    def dilatation(self,dilatation_size,shape,image):

        dilation_shape = self.morph_shape(shape)

        element = cv.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),(dilatation_size, dilatation_size))

        dilatation_dst = cv.dilate(image, element)
        return dilatation_dst




    def convert_depth_frame_to_pointcloud(self, depth_image, camera_intrinsics ):
        """
        Convert the depthmap to a 3D point cloud
        Parameters:
        -----------
        depth_frame 	 	 : rs.frame()
                            The depth_frame containing the depth map
        camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
        Return:
        ----------
        x : array
            The x values of the pointcloud in meters
        y : array
            The y values of the pointcloud in meters
        z : array
            The z values of the pointcloud in meters
        """

        [height, width] = depth_image.shape

        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        x = (u.flatten() - camera_intrinsics.ppx)/camera_intrinsics.fx
        y = (v.flatten() - camera_intrinsics.ppy)/camera_intrinsics.fy

        z = depth_image.flatten() / 1000
        x = np.multiply(x,z)
        y = np.multiply(y,z)

        # x = x[np.nonzero(z)]
        # y = y[np.nonzero(z)]
        # z = z[np.nonzero(z)]

        x = np.reshape(x,(height,width))
        y = np.reshape(y,(height,width))
        z = np.reshape(z,(height,width))


        return x,y,z

    # this function takes the input img and returns the skeletonization of the image.
    def skeleton(self,img):
        # Step 1: Create an empty skeleton
        size = np.size(img)
        skel = np.zeros(img.shape, np.uint8)

        # Get a Cross Shaped Kernel
        element = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))

        # Repeat steps 2-4
        while True:
            #Step 2: Open the image
            open = cv.morphologyEx(img, cv.MORPH_OPEN, element)
            #Step 3: Substract open from the original image
            temp = cv.subtract(img, open)
            #Step 4: Erode the original image and refine the skeleton
            eroded = cv.erode(img, element)
            skel = cv.bitwise_or(skel,temp)
            img = eroded.copy()
            # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
            if cv.countNonZero(img)==0:
                break

        return skel


    # this function generates a normal vector form the given point cloud.
    def pythoMath(self,x,y,z):


        xs = x.flatten()[::40]
        ys = y.flatten()[::40]
        zs = z.flatten()[::40]

        xs = [i for i in xs if i != 0]
        ys = [i for i in ys if i != 0]
        zs = [i for i in zs if i != 0]
        if len(xs)<3:
            return

        # # plot raw data
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(xs, ys, zs, color='b')

        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(xs)):
            tmp_A.append([xs[i], ys[i], 1])
            tmp_b.append(zs[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)

        # Manual solution
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)

        # Or use Scipy
        # from scipy.linalg import lstsq
        # fit, residual, rnk, s = lstsq(A, b)

        ##### NORMAL VECTOR FOR THE PLANE ####
        # print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        ######################################
        # print("errors: \n", errors)
        # print("residual:", residual)

        # # plot plane
        # xlim = ax.get_xlim()
        # ylim = ax.get_ylim()
        # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
        #                 np.arange(ylim[0], ylim[1]))
        # Z = np.zeros(X.shape)
        # for r in range(X.shape[0]):
        #     for c in range(X.shape[1]):
        #         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
        #         # print(Z[r,c])
        # ax.plot_wireframe(X,Y,Z, alpha=1)
        # # ax.plot_surface(X,Y,Z,alpha=0.2)

        # # print(X,Y)
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')
        # ax.set_zlabel('z')
        # plt.show()
        return fit



    def dot_product(self,x, y):
        return sum([x[i] * y[i] for i in range(len(x))])

    def norm(self, x):
        return sqrt(self.dot_product(x, x))

    def normalize(self,x):
        return [x[i] / self.norm(x) for i in range(len(x))]

    # this function projects the vector onto the plane.
    def project_onto_plane(self,x, n):
        d = self.dot_product(x, n) / self.norm(n)
        p = [d * self.normalize(n)[i] for i in range(len(n))]
        return [x[i] - p[i] for i in range(len(x))]


    # this function runs the whole program every timer_period seconds.
    def handtracker_callback(self):
        try:
            print("___________Beginning________________")
            
            frames = self.pipeline.wait_for_frames()# Get frameset of color and depth
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                # continue
                return

            depth_image = np.asanyarray(aligned_depth_frame.get_data()) # depth image is a 1 channel image
            color_image = np.asanyarray(color_frame.get_data()) # color image is a 3 channel image



            # this is to remove the background from the depth image. and to make the depth a 3 channel image. 
            # Set pixels further than self.clipping_distance to grey
            grey_color = 0
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) # depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 200), grey_color, color_image)

            # make bg_removed as a binary image. And is only used to visulatiation.
            binary_bg_removed = cv.cvtColor(bg_removed,cv.COLOR_RGB2GRAY)


            # takes the removed background image and converts it to HSV
            HSV_image = cv.cvtColor(bg_removed,cv.COLOR_RGB2HSV)

            #############################################
            ####    HSV THRESHOLD VALUES FOR BLUE    ####
            lower_blue = np.array([10,40,70])
            upper_blue = np.array([30,255,255])
            #############################################

            # takes the HSV image and thresholds it to get the blue color, and makes a binary image, with 1 channel.
            H_image = cv.inRange(HSV_image,lower_blue,upper_blue)

            # makes the binary image with 3 channel image.
            H_image_3d = np.dstack((H_image,H_image,H_image))


            # this is to get the depth of the blue pixels.
            H_image_depth = np.where(H_image !=0 ,  depth_image,grey_color)

            # this is to convert blue pixels depth to a point cloud.
            x,y,z = self.convert_depth_frame_to_pointcloud(H_image_depth,self.camera_intrinsics)

            # this is to get the normal vector of the plane.
            Z_axis_vector = self.pythoMath(x,y,z)

            # we dialate and erode the binary image to get a better skeleton.
            closing = self.dilatation(40,2,H_image)
            closing = self.erosion(80,2,closing)

            # this is to get the skeleton of the blue pixels.
            skel = self.skeleton(closing)
            X,Y = np.where(skel==255) # this is to get the x and y coordinates of the skeleton pixels.

            # checks if there are enough points to fit a line.
            if len(X)<3:
                return

            a,b = np.polyfit(X,Y,1)
            # plt.scatter(X,Y)
            # plt.plot(X,a*X+b)


            # we define a vector that span the x and y plane.
            vector = [1,a,0]
            # then the vector is projected onto the plane of the hand.
            Y_axis_vector = self.project_onto_plane(vector,Z_axis_vector)
            Y_axis_vector = np.array([Y_axis_vector[0][0,0],Y_axis_vector[1][0,0],Y_axis_vector[2][0,0]]) # this is to convert the projected vector to a numpy array.
            Z_axis_vector = np.array([Z_axis_vector[0,0],Z_axis_vector[1][0,0],Z_axis_vector[2][0,0]]) # this it to convert the normal vector to a numpy array.

            # we then take the cross product of the Y and Z vectors to get the X axis vector.
            X_axis_vector = np.array(np.cross(Y_axis_vector,Z_axis_vector)) 

            # we normalize all the axis vectors.
            self.X_axis_normalized = cv.normalize(X_axis_vector, (0,1))
            self.Y_axis_normalized = cv.normalize(Y_axis_vector, (0,1))
            self.Z_axis_normalized = cv.normalize(Z_axis_vector, (0,1))


            # we make a small dialation to the thresholded blue glove image.
            distTrans = self.dilatation(10,2,H_image)

            # then the dialated image is used to find the max values in the image.
            distTrans = cv.distanceTransform(distTrans, cv.DIST_L2, 3)

            ########################################################
            ### this is to shows the distance transform image ######
            ########################################################

            cv.namedWindow('distance transform', cv.WINDOW_NORMAL)
            cv.imshow('distance transform', distTrans)

            ########################################################


            # this finds the max values in the dialated image.
            max_Value_Depth = np.amax(distTrans)

            # gets the coordinates of the max values in dialated depth.
            XY_max_Depth_Loc = np.where(distTrans==max_Value_Depth)

            # gets the first max value coordinate from dialated depth.
            isolate_XY_max_Depth_Loc = [XY_max_Depth_Loc[1][0],XY_max_Depth_Loc[0][0]]

            # creates a circle around the first max value in dialated depth. This is done for vizualitaion.
            cv.circle(distTrans,isolate_XY_max_Depth_Loc,20,255,thickness=3)

            # generates a point cloud from the depth image.
            x,y,z = self.convert_depth_frame_to_pointcloud(depth_image,self.camera_intrinsics)
            xyz = np.dstack((x,y,z)) # this is to stack the x,y,z arrays into a 3d array.

            # normalizes the dialated depth image to be between 0 and 1. so that it can be shown. This is done for vizualitaion.
            distTrans = cv.normalize(distTrans,distTrans,0,1.0,cv.NORM_MINMAX)



            # find the 3d coordinates of the max value in the dialated depth image. also known as the asumed middle of the hand.
            hand_coor = xyz[isolate_XY_max_Depth_Loc[1]][isolate_XY_max_Depth_Loc[0]]

            # this defines the transformation matrix of the hand.
            transform_hand =  [[self.X_axis_normalized[0,0],self.Y_axis_normalized[0,0],self.Z_axis_normalized[0,0],hand_coor[0]],
                               [self.X_axis_normalized[1,0],self.Y_axis_normalized[1,0],self.Z_axis_normalized[1,0],hand_coor[1]],
                               [self.X_axis_normalized[2,0],self.Y_axis_normalized[2,0],self.Z_axis_normalized[2,0],hand_coor[2]],
                               [0                          ,0                          ,0                          ,1           ]]


            # this turns the rotation matrix in the transformation matrix into a quaternion.
            rotation_hand = tf.quaternion_from_matrix(transform_hand)

            # this creates a pose message to be published.
            hand_pose = Pose()

            # this fills the pose message with the correct values.
            hand_pose._orientation._x = rotation_hand[0]
            hand_pose._orientation._y = rotation_hand[1]
            hand_pose._orientation._z = rotation_hand[2]
            hand_pose._orientation._w = rotation_hand[3]
            hand_pose._position._x = transform_hand[0][3]*-1
            hand_pose._position._y = transform_hand[1][3]*-1
            hand_pose._position._z = transform_hand[2][3]



            #####################################################################################
            #######      see if hand is in one place             ################################
            #####################################################################################

            # starts the timer
            self.when_to_start_time +=1
            if self.when_to_start_time == 1:
                self.hand_start_time = time.time()

            # this saves the position of the hand for the last 5 iterations.
            self.old_positions[self.iteration] = [transform_hand[0][3]*-1,transform_hand[1][3]*-1,transform_hand[2][3]]
            self.iteration+= 1

            # this finds the distance between the hands positions. 
            dist1 = np.linalg.norm(self.old_positions[0]-self.old_positions[1])
            dist2 = np.linalg.norm(self.old_positions[1]-self.old_positions[2])
            dist3 = np.linalg.norm(self.old_positions[2]-self.old_positions[3])
            dist4 = np.linalg.norm(self.old_positions[4]-self.old_positions[0])

            # this finds the average distance between the hands positions.
            dist_ave = (dist1 +dist2 + dist3 + dist4)/4

            # this resets the iteration counter.
            if self.iteration >= self.old_positions.shape[0]:
                self.iteration=0

            # this checks if the hands position has changed more than 5 cm in the last 5 iterations.
            if dist_ave > 0.05:
                return



            #####################################################################################
            #######      see if the blob is right size             ##############################
            #####################################################################################

            # find the blob size from the dialated thresholded hue image
            blob_size = 0
            for row in closing:
                for element in row:
                    if element >=1:
                        blob_size +=1


            # calibratede constants for exponential function
            a = 270630.6961550408
            b = -7.466334315269839

            # finds the theoretical blob size to the dialated and eroded thresholded hue image
            theoretical_blob_size = a * np.exp(b * hand_coor[2])

            # finds the difference between the theoretical blob size and the actual blob size
            blob_differeants = theoretical_blob_size-blob_size

            # finds the procentage difference between the theoretical blob size and the actual blob size
            procentage_diff = (abs(blob_differeants)/(theoretical_blob_size+blob_size)/2)*100
            print("procentage diff: ", procentage_diff)

            # if the blob is the right size, then the hand pose will be published.
            if procentage_diff <=20:
                self.pub_pose.publish(hand_pose) # publishes the hand pose
                self.searchBool = False # makes so the detection algorithm will not run again until the robot has moved back to the starting position.

                hand_end_time = time.time() 

                # prints the time it took to detect the hand.
                time_diff = hand_end_time - self.hand_start_time
                print("how long to detect hand: ", time_diff)

                # resets the timer
                self.when_to_start_time=0

            # says if the blob is to big or to small
            if blob_differeants < 0:
                print("blob diff: To big")
            else:
                print("blob diff: To small")


            ########### calibration of hans size ############
            # self.blob_depth.append(hand_coor[2])
            # self.blob_size1.append(blob_size)

            # self.x = self.blob_depth
            # self.y = self.blob_size1

            # self.getdata += 1
            # if self.getdata > 100:

            #     # # plot raw data
            #     # plt.figure()
            #     # ax = plt.subplot(111)
            #     # ax.scatter(x, y, color='b')
            #     plt.scatter(self.x, self.y, color="red", marker="o")
            #     plt.show()

            # print("blob depth: ", self.blob_depth)
            # print("blob size: ",self.blob_size1)

            #####################################################################################
            #######      showing images                          ################################
            #####################################################################################

            H_image_depth_3d = np.dstack((H_image_depth,H_image_depth,H_image_depth))
            H_image_depth_3d = H_image_depth_3d.astype(np.uint8)

            # makes the depth images 
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

            # this shows the processed images for each step in the detection algorithm.
            cv.namedWindow('depth_colormap', cv.WINDOW_NORMAL)
            cv.imshow('depth_colormap', depth_colormap)

            cv.namedWindow('RGB', cv.WINDOW_NORMAL)
            cv.imshow('RGB', color_image)
            
            cv.namedWindow('binary_bg_removed', cv.WINDOW_NORMAL)
            cv.imshow('binary_bg_removed', binary_bg_removed)
            
            cv.namedWindow('bg_removed', cv.WINDOW_NORMAL)
            cv.imshow('bg_removed', bg_removed)
            
            cv.namedWindow('skeleton', cv.WINDOW_NORMAL)
            cv.imshow('skeleton', skel)
            
            cv.namedWindow('big dialated and eroded', cv.WINDOW_NORMAL)
            cv.imshow('big dialated and eroded', closing)
            
            cv.namedWindow('hue', cv.WINDOW_NORMAL)
            cv.imshow('hue', H_image_3d)
            
            cv.namedWindow('distance transform', cv.WINDOW_NORMAL)
            cv.imshow('distance transform', distTrans)


            print("_________________End________________")

        except:
            traceback.print_exc()
            return





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
