# for subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

# for publisher
import cv2
from sensor_msgs.msg import Image # Image is the message type

class ImageListener(Node):
    def __init__(self, depth_image_topic, depth_info_topic):
        super().__init__('show_center_depth')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageColorCallback, 1)
        # self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)

        self.pub = self.create_publisher(msg_Image, 'video_frames', 1)

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None


    def imageColorCallback(self, data):
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image))



    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            pix = (indices[1], indices[0])
            self.pix = pix
            # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[1], pix[0], cv_image[pix[1], pix[0]])

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return





    # def imageDepthInfoCallback(self, cameraInfo):
    #     try:
    #         if self.intrinsics:
    #             return
    #         self.intrinsics = rs2.intrinsics()
    #         self.intrinsics.width = cameraInfo.width
    #         self.intrinsics.height = cameraInfo.height
    #         self.intrinsics.ppx = cameraInfo.k[2]
    #         self.intrinsics.ppy = cameraInfo.k[5]
    #         self.intrinsics.fx = cameraInfo.k[0]
    #         self.intrinsics.fy = cameraInfo.k[4]
    #         if cameraInfo.distortion_model == 'plumb_bob':
    #             self.intrinsics.model = rs2.distortion.brown_conrady
    #         elif cameraInfo.distortion_model == 'equidistant':
    #             self.intrinsics.model = rs2.distortion.kannala_brandt4
    #         self.intrinsics.coeffs = [i for i in cameraInfo.d]
    #     except CvBridgeError as e:
    #         print(e)
    #         return




class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'video_frames', 1)

    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()

    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    # Display the message on the console
    self.get_logger().info('Publishing video frame')


def main(args=None):

    # # Initialize the rclpy library
    # rclpy.init(args=args)

    # # Create the node
    # image_publisher = ImagePublisher()

    # # Spin the node so the callback function is called.
    # rclpy.spin(image_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # image_publisher.destroy_node()

    # # Shutdown the ROS client library for Python
    # rclpy.shutdown()

    # Initialize the rclpy library
    rclpy.init(args=args)


    #sets the topics to subscribe to
    color_image_topic = '/camera/color/image_raw'
    depth_image_topic = '/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/depth/camera_info'

    # print ()
    # print ('show_center_depth.py')
    # print ('--------------------')
    # print ('App to demontrate the usage of the /camera/depth topics.')
    # print ()
    # print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    # print ('Application then calculates and print the range to the closest object.')
    # print ('If intrinsics data is available, it also prints the 3D location of the object')
    # print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    # print ()

    # Create the node
    listener = ImageListener(color_image_topic, depth_info_topic)

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
