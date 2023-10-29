
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray
import numpy as np

class ReplicaRGBDSubscriber(Node):
    def __init__(self):
        super().__init__('replica_rgbd_subscriber')
        self.subscription_rgb = self.create_subscription(Image, '/camera/rgb/image_raw', self.rgb_image_callback, 10)
        self.subscription_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 10)
        # self.subscription_camera_info = self.create_subscription(CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10)
        self.subscription_depth = self.create_subscription(Float32MultiArray, '/camera/twc', self.twc_callback, 10)
        self.bridge = CvBridge()

    def rgb_image_callback(self, msg):
        # 将ROS 2 Image消息转换为OpenCV图像
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 在这里可以处理RGB图像，例如显示或保存

        # 播报成功接收帧的信息
        self.get_logger().info("成功接收RGB图像帧")

    def depth_image_callback(self, msg):
        # 将ROS 2 Image消息转换为OpenCV图像
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

        # 在这里可以处理深度图像，例如显示或保存

        # 播报成功接收帧的信息
        self.get_logger().info("成功接收深度图像帧")

    def twc_callback(self, msg):
        # twc = self.msg
        twc = np.array(msg.data).reshape((4, 4))
        print(twc)

    # def camera_info_callback(self, msg):
        # 在这里可以处理相机信息，例如根据相机参数进行深度图像处理
        # fx = msg.K[0]  # 焦距 X
        # fy = msg.width  # 焦距 Y
        # cx = msg.K[2]  # 光学中心 X
        # cy = msg.K[5]  # 光学中心 Y
        # camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        # camera_info_ = rgb_image.shape[1]
        # camera_info_msg.height = rgb_image.shape[0]

def main(args=None):
    rclpy.init(args=args)
    node = ReplicaRGBDSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

