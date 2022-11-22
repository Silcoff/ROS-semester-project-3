## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2 as cv



class Handtracker():
    def __init__(self):
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
        clipping_distance_in_meters = 0.8 #1 meter
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)




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

        return x, y, z

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


    def handtracker_callback(self):

        # Streaming loop
        try:
            while True:
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
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                lowerboundDepth = 200
                upperboundDepth = 650
                # threshold depth image
                ret, binary_depth_image = np.array(cv.threshold(depth_image,lowerboundDepth,upperboundDepth,cv.THRESH_TOZERO))
                ret, binary_depth_image = np.array(cv.threshold(binary_depth_image,upperboundDepth,upperboundDepth,cv.THRESH_TOZERO_INV))

                # convert to binary
                ret, binary_depth_image = np.array(cv.threshold(binary_depth_image,0,upperboundDepth,cv.THRESH_BINARY))


                # Remove background - Set pixels further than self.clipping_distance to grey
                grey_color = 0
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
                bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 00), grey_color, color_image)

                HSV_image = cv.cvtColor(bg_removed,cv.COLOR_RGB2HSV)

                H_image = HSV_image[:,:,1]

                ret, H_image = cv.threshold(H_image,160,255,cv.THRESH_BINARY)
                ret, H_image = cv.threshold(H_image,170,255,cv.THRESH_BINARY_INV)
                ret, H_image = cv.threshold(H_image,170,255,cv.THRESH_BINARY_INV)

                # H_image = np.where(H_image==0,255,0)
                # print(H_image)
                H_image_3d = np.dstack((H_image,H_image,H_image))

                bg_removed_grayscale = cv.cvtColor(bg_removed,cv.COLOR_RGB2GRAY)
                ret, binary_bg_removed = cv.threshold(bg_removed_grayscale,0,255,cv.THRESH_BINARY)

                closing = self.dilatation(3,2,binary_bg_removed)
                closing = self.erosion(6,2,closing)
                # closing = self.dilatation(3,2,closing)

                skel =self.skeleton(closing)

                binary_bg_removed = np.dstack((binary_bg_removed,binary_bg_removed,binary_bg_removed))

                distTrans = cv.distanceTransform(closing, cv.DIST_L2, 3)

                # find max values in dialated depth
                max_Value_Depth = np.amax(distTrans)

                # compute all places where max value in dialeted depth is
                XY_max_Depth_Loc = np.where(distTrans==max_Value_Depth)

                # get fisrt max value coordinate from dialated depth
                isolate_XY_max_Depth_Loc = [XY_max_Depth_Loc[1][0],XY_max_Depth_Loc[0][0]]

                # create circle around max value in dialated depth
                cv.circle(distTrans,isolate_XY_max_Depth_Loc,20,255,thickness=3)


                # find depth computed from the max value coordinates
                hand_depth = depth_image[isolate_XY_max_Depth_Loc[1]][isolate_XY_max_Depth_Loc[0]]/1000
                # print(hand_depth)

                x,y,z = self.convert_depth_frame_to_pointcloud(depth_image,self.camera_intrinsics)

                xyz= np.dstack((x,y,z))
                distTrans = cv.normalize(distTrans,distTrans,0,1.0,cv.NORM_MINMAX)
                distTrans = np.dstack((distTrans,distTrans,distTrans))
                hand_coor = xyz[isolate_XY_max_Depth_Loc[1]][isolate_XY_max_Depth_Loc[0]]
                # print(hand_coor)

                skel_3d = np.dstack((skel,skel,skel))
                hue=H_image_3d[isolate_XY_max_Depth_Loc[1]][isolate_XY_max_Depth_Loc[0]]
                # print(hue)
                binary_depth_image_3d = np.dstack((binary_depth_image,binary_depth_image,binary_depth_image))
                closing_3d = np.dstack((closing,closing,closing))
                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
                images = np.hstack((depth_colormap,binary_bg_removed,closing_3d,skel_3d))
                cv.namedWindow('Align Example', cv.WINDOW_NORMAL)
                cv.imshow('asdf',binary_depth_image_3d)
                cv.imshow('Align Example', images)
                key = cv.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()



object = Handtracker()

object.handtracker_callback()