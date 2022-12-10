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
import traceback


from matplotlib import pyplot as plt

from std_msgs.msg import Bool



# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2 as cv
from math import sqrt
import traceback



from geometry_msgs.msg import Pose

import tf_transformations as tf

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np




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
        self.pub_bgr = self.create_publisher(msg_Image,"/rgb_image",1)

        self.pub_depth = self.create_publisher(msg_Image,"/depth_image",1)
        
        # we create a publisher, where the first intput is the msg type, seconde is the topic name, last is unkown
        self.pub_pose = self.create_publisher(Pose, 'hand_pose_msg', 1)
        self.create_subscription(Bool, 'searchBool',self.change_Bool_callback,1)

        self.old_positions = np.zeros((5,3))

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.handtracker_callback)

        self.iteration = 0

        self.when_to_start_time = 0

        #####################################
        #####     test blob size   ##########
        #####################################

        self.blob_depth = []
        self.blob_size1 = []
        self.getdata = 0

        ##############################################
                # Create a self.pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the self.pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        profile_depth = profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
        self.camera_intrinsics  = profile_depth.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics


        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 0.65 #1 meter
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)



    def change_Bool_callback(self,data):
        self.searchBool = data.data
        print(self.searchBool)



    def morph_shape(self,val):
        if val == 0:
            return cv.MORPH_RECT
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

    def pythoMath(self,x,y,z):

        # if x.all()==0:
        #     return

        # These constants are to create random data for the sake of this example
        N_POINTS = 10
        TARGET_X_SLOPE = 2
        TARGET_y_SLOPE = 3
        TARGET_OFFSET  = 5
        EXTENTS = 5
        NOISE = 5

        # Create random data.
        # In your solution, you would provide your own xs, ys, and zs data.
        # xs = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
        # ys = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
        # zs = []
        xs = x.flatten()[::40]
        ys = y.flatten()[::40]
        zs = z.flatten()[::40]
        
        xs = [i for i in xs if i != 0]
        ys = [i for i in ys if i != 0]
        zs = [i for i in zs if i != 0]
        if len(xs)<3:
            return        
        # print(xs,ys,zs)

        # for i in range(N_POINTS):
        #     zs.append(xs[i]*TARGET_X_SLOPE + \
        #             ys[i]*TARGET_y_SLOPE + \
        #             TARGET_OFFSET + np.random.normal(scale=NOISE))

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

    def project_onto_plane(self,x, n):
        d = self.dot_product(x, n) / self.norm(n)
        p = [d * self.normalize(n)[i] for i in range(len(n))]
        return [x[i] - p[i] for i in range(len(x))]


    def handtracker_callback(self):
        # # print("handtracker")
        if self.searchBool == False:
            # print(self.searchBool)
            return
        # Streaming loop
        try:
                print("___________Beginning________________")
            # while True:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
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

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())



                # Remove background - Set pixels further than self.clipping_distance to grey
                grey_color = 0
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
                bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 200), grey_color, color_image)

                depth_bg_removed = np.where((depth_image > self.clipping_distance) | (depth_image <= 200), grey_color, depth_image)

                depth_bg_removed = depth_bg_removed.astype(np.uint8)
                HSV_image = cv.cvtColor(bg_removed,cv.COLOR_RGB2HSV)

                #### USED TO CALIBRATE THE HSV FOR THE BLUE GLOVE ######
                # print(HSV_image[320,240])
                ########################################################

                lower_blue = np.array([10,90,100])
                upper_blue = np.array([25,255,255])

                H_image = cv.inRange(HSV_image,lower_blue,upper_blue)


                H_image_3d = np.dstack((H_image,H_image,H_image))

                H_image_depth = np.where(H_image !=0 ,  depth_image,grey_color)
                x,y,z = self.convert_depth_frame_to_pointcloud(H_image_depth,self.camera_intrinsics)
                # XYZ_H_image_depth = np.dstack(x,y,z)

                Z_axis_vector = self.pythoMath(x,y,z)
                # normal = self.plane_from_points(XYZ_H_image_depth)
                # if (normal == None).all() :
                #     return
                # print(normal)    
                bg_removed_grayscale = cv.cvtColor(bg_removed,cv.COLOR_RGB2GRAY)
                ret, binary_bg_removed = cv.threshold(bg_removed_grayscale,0,255,cv.THRESH_BINARY)

                closing = self.dilatation(40,2,H_image)
                closing = self.erosion(80,2,closing)
                # closing = self.dilatation(3,2,closing)

                skel = self.skeleton(closing)
                X,Y = np.where(skel==255)

                if len(X)<3:
                    return
                
                a,b = np.polyfit(X,Y,1)
                # plt.scatter(X,Y)
                # plt.plot(X,a*X+b)



                vector = [1,a,0]
                Y_axis_vector = self.project_onto_plane(vector,Z_axis_vector)
                Y_axis_vector = np.array([Y_axis_vector[0][0,0],Y_axis_vector[1][0,0],Y_axis_vector[2][0,0]])
                Z_axis_vector = np.array([Z_axis_vector[0,0],Z_axis_vector[1][0,0],Z_axis_vector[2][0,0]])

                # print("y vector", Y_axis_vector)
                # print("Z vector", Z_axis_vector)
                dotYZ= np.dot(Y_axis_vector,Z_axis_vector)
                X_axis_vector = np.array(np.cross(Y_axis_vector,Z_axis_vector))
                # print("x vector", X_axis_vector)
                dotXZ= np.dot(X_axis_vector,Z_axis_vector)
                dotYX= np.dot(Y_axis_vector,X_axis_vector)

                
                # print("check orthoganallity YX: " , dotYZ)
                # print("check orthoganallity XZ: " , dotXZ)
                # print("check orthoganallity YX: " , dotYX)

                self.X_axis_normalized = cv.normalize(X_axis_vector, (0,1))
                self.Y_axis_normalized = cv.normalize(Y_axis_vector, (0,1))
                self.Z_axis_normalized = cv.normalize(Z_axis_vector, (0,1))




                binary_bg_removed = np.dstack((binary_bg_removed,binary_bg_removed,binary_bg_removed))

                # distTrans = cv.distanceTransform(closing, cv.DIST_L2, 3)
                distTrans = cv.distanceTransform(H_image, cv.DIST_L2, 3)

                # find max values in dialated depth
                max_Value_Depth = np.amax(distTrans)

                # compute all places where max value in dialeted depth is
                XY_max_Depth_Loc = np.where(distTrans==max_Value_Depth)
                # print(XY_max_Depth_Loc)
                # get fisrt max value coordinate from dialated depth
                isolate_XY_max_Depth_Loc = [XY_max_Depth_Loc[1][0],XY_max_Depth_Loc[0][0]]

                # create circle around max value in dialated depth
                cv.circle(distTrans,isolate_XY_max_Depth_Loc,20,255,thickness=3)


                x,y,z = self.convert_depth_frame_to_pointcloud(depth_image,self.camera_intrinsics)
                xyz = np.dstack((x,y,z))
                distTrans = cv.normalize(distTrans,distTrans,0,1.0,cv.NORM_MINMAX)
                distTrans = np.dstack((distTrans,distTrans,distTrans))
                # find hand x,y computed from the max value coordinates
                hand_coor = xyz[isolate_XY_max_Depth_Loc[1]][isolate_XY_max_Depth_Loc[0]]
                # print(hand_coor)

                transform_hand =  [[self.X_axis_normalized[0,0],self.Y_axis_normalized[0,0],self.Z_axis_normalized[0,0],hand_coor[0]],
                                   [self.X_axis_normalized[1,0],self.Y_axis_normalized[1,0],self.Z_axis_normalized[1,0],hand_coor[1]],
                                   [self.X_axis_normalized[2,0],self.Y_axis_normalized[2,0],self.Z_axis_normalized[2,0],hand_coor[2]],
                                   [0                          ,0                          ,0                          ,1           ]]
                
                # print("matrix rotiation: " ,rotation_hand)
                rotation_hand = tf.quaternion_from_matrix(transform_hand)
                # print("quaternion: " ,rotation_hand)
                
                hand_pose = Pose()

                hand_pose._orientation._x = rotation_hand[0]
                hand_pose._orientation._y = rotation_hand[1]
                hand_pose._orientation._z = rotation_hand[2]
                hand_pose._orientation._w = rotation_hand[3]

                hand_pose._position._x = transform_hand[0][3]*-1
                hand_pose._position._y = transform_hand[1][3]*-1
                hand_pose._position._z = transform_hand[2][3]

                # print("x: " ,transform_hand[0][3])
                # print("y: " ,transform_hand[1][3])
                # print("z: " ,transform_hand[2][3])



               


                #####################################################################################
                #######      see if hand is in one place             ################################
                #####################################################################################

                self.when_to_start_time +=1    
                if self.when_to_start_time == 1:
                    self.hand_start_time = time.time()

                print(self.iteration) 
                self.old_positions[self.iteration] = [transform_hand[0][3]*-1,transform_hand[1][3]*-1,transform_hand[2][3]]
                # print(self.old_positions)
                self.iteration+= 1

                dist1 = np.linalg.norm(self.old_positions[0]-self.old_positions[1])
                dist2 = np.linalg.norm(self.old_positions[1]-self.old_positions[2])
                dist3 = np.linalg.norm(self.old_positions[2]-self.old_positions[3])
                dist4 = np.linalg.norm(self.old_positions[4]-self.old_positions[0])

                dist_ave = (dist1 +dist2 + dist3 + dist4)/4
                # print(dist_ave)
                if self.iteration >= self.old_positions.shape[0]:
                    self.iteration=0
                    
                
                if dist_ave > 0.05:
                    return
                    
                    # self.pub_pose.publish(hand_pose)
                    # self.searchBool = False



                #####################################################################################
                #######      see if the blob is right size             ##############################
                #####################################################################################


                blob_size = 0
                for row in closing:
                    for element in row:
                        if element >=1:
                            blob_size +=1
                
                # print("blob size: ", blob_size)
                # print("hand depth: ",hand_coor[2])



                #calibratede konstants for exponential function
                # old_a = 486734.03262152773
                # old_b = -9.231364249894371
                a = 337990.8954294294
                b = -10.985894194395941

                theoretical_blob_size = a * np.exp(b * hand_coor[2])

                blob_differeants = theoretical_blob_size-blob_size

                procentage_diff = (abs(blob_differeants)/(theoretical_blob_size+blob_size)/2)*100
                print("procentage diff: ",procentage_diff)

                if procentage_diff <=20:
                    self.pub_pose.publish(hand_pose)
                    self.searchBool = False
                    hand_end_time = time.time()

                    time_diff = hand_end_time - self.hand_start_time
                    print("how long to detect hand: ", time_diff)
                    self.when_to_start_time=0

                    
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
                
                # print(transform_hand)    
                skel_3d = np.dstack((skel,skel,skel))
                # binary_depth_image_3d = np.dstack((binary_depth_image,binary_depth_image,binary_depth_image))
                closing_3d = np.dstack((closing,closing,closing))
                
                H_image_depth_3d = np.dstack((H_image_depth,H_image_depth,H_image_depth))
                H_image_depth_3d = H_image_depth_3d.astype(np.uint8)
                # Render images:
                #   depth align to color on left
                #   depth on right
                

                depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
                images = np.hstack((bg_removed,H_image_3d))

                cv.namedWindow('Align Example', cv.WINDOW_NORMAL)
                cv.imshow('Align Example', images)
                # cv.imshow('asdf',depth_image)
                key = cv.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv.destroyAllWindows()
                    # break
                # self.searchBool = False
                print("_________________End________________")

        except:
            traceback.print_exc()
            return


        finally:
            print()
            # self.pipeline.stop()




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
