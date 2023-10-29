import rclpy
import message_filters
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray
import numpy as np

class Test:
    def __init__(self):
        super.__init__()
        print('initin')

    def callback(self,color,depth,twc):
        bridge = CvBridge
        color_image = bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth, desired_encoding="16UC1")
        twc_image = np.array(twc.data).reshape((4, 4))
        print(twc)


    def run(self):
        print("qidong")
        rclpy.init_node('listener', anonymous=True)
        # realsense
        color_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        pose_sub = message_filters.Subscriber('/camera/twc', Float32MultiArray)

        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, pose_sub], 2, 1 / 10,
                                                             allow_headerless=True)
        print(" ========== MAPPING START ===========")
        ts.registerCallback(self.callback)
        rclpy.spin()

# def main(args=None):
    # t = Test()
    # t.run()

# def main(args=None):
    # rclpy.init(args=args)
    # test = Test()
    # test.run()
    # rclpy.shutdown()

# if __name__ == '__main__':
    # main()

class map:
    def __init__(self):
        self.test = Test()

    def run(self):
        self.test.run(self)


def main():
    t = map
    t.run()