# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import os

# class ReplicaDepthPublisher(Node):
#     def __init__(self, dataset_path):
#         super().__init__('replica_depth_publisher')
#         self.publisher_ = self.create_publisher(Image, '/camera/depth/image_raw', 10)
#         self.timer_ = self.create_timer(1.0, self.publish_depth_image)  # 1Hz timer
#         self.dataset_path = dataset_path
#         self.bridge = CvBridge()

#     def publish_depth_image(self):
#         # 读取深度图像文件
#         depth_image_file = os.path.join(self.dataset_path, 'depth', f'{self.timer_.count:04d}.png')
#         if os.path.exists(depth_image_file):
#             depth_image = cv2.imread(depth_image_file, cv2.IMREAD_UNCHANGED)

#             # 创建ROS 2 Image消息并填充深度图像数据
#             depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
#             depth_image_msg.header.stamp = self.get_clock().now().to_msg()

#             # 发布深度图像
#             self.publisher_.publish(depth_image_msg)
#         else:
#             self.get_logger().warning(f"Depth image file not found: {depth_image_file}")

# def main(args=None):
#     rclpy.init(args=args)
#     # dataset_path = '/path/to/replica_dataset'
#     dataset_path = '/home/yang/office_0/imap/00'
#     node = ReplicaDepthPublisher(dataset_path)
#     rclpy.spin(node)
#     rclpy.shutdown()

# # if __name__ == '__main__':
#     # main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import os

# class ReplicaDepthPublisher(Node):
#     def __init__(self, dataset_path):
#         super().__init__('replica_depth_publisher')
#         self.publisher_ = self.create_publisher(Image, '/camera/depth/image_raw', 10)
#         self.timer_ = self.create_timer(1.0, self.publish_depth_image)  # 1Hz timer
#         self.dataset_path = dataset_path
#         self.bridge = CvBridge()
#         self.frame_count = 0  # 用于跟踪帧计数

#     def publish_depth_image(self):
#         # 构建深度图像文件的完整路径
#         depth_image_file = os.path.join(self.dataset_path, 'depth', f'{self.frame_count:04d}.png')
        
#         if os.path.exists(depth_image_file):
#             # 读取深度图像数据
#             depth_image = cv2.imread(depth_image_file, cv2.IMREAD_UNCHANGED)

#             # 创建ROS 2 Image消息并填充深度图像数据
#             depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
#             depth_image_msg.header.stamp = self.get_clock().now().to_msg()

#             # 发布深度图像
#             self.publisher_.publish(depth_image_msg)
            
#             self.frame_count += 1  # 更新帧计数

# def main(args=None):
#     rclpy.init(args=args)
#     dataset_path = '/home/yang/office_0/imap/00'
#     node = ReplicaDepthPublisher(dataset_path)
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from std_msgs.msg import Float32MultiArray

class ReplicaRGBDPublisher(Node):
    def __init__(self, dataset_path):
        super().__init__('replica_rgbd_publisher')
        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.publisher_T = self.create_publisher(Float32MultiArray, '/camera/twc', 10)
        # self.publisher_camera_info = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        self.timer = self.create_timer(0.1, self.publish_rgbd_image)  # 1Hz timer
        self.dataset_path = dataset_path
        self.bridge = CvBridge()   # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        self.frame_count = 0  # 用于跟踪帧计数

    def publish_rgbd_image(self):
        # 构建RGB图像文件和深度图像文件的完整路径
        rgb_image_file = os.path.join(self.dataset_path, 'rgb', f'rgb_{self.frame_count}.png')
        print(rgb_image_file)
        depth_image_file = os.path.join(self.dataset_path, 'depth', f'depth_{self.frame_count}.png')
        print(depth_image_file)
        # TODO
        traj_file = os.path.join(self.dataset_path, "traj_w_c.txt")
        # 加载txt文件，获取Twc和depth_transform
        # self.Twc = np.loadtxt(traj_file, delimiter=" ").reshape([-1, 4, 4])
        # rgb_image_twc = os.

        
        if os.path.exists(rgb_image_file) and os.path.exists(depth_image_file):
            # 读取RGB图像和深度图像数据
            rgb_image = cv2.imread(rgb_image_file)
            depth_image = cv2.imread(depth_image_file, cv2.IMREAD_UNCHANGED)

            Twc = np.loadtxt(traj_file, delimiter=" ").reshape([-1, 4, 4])
            self.get_logger().info("tupiandaoru")
            

            # 创建ROS 2 Image消息并填充RGB图像数据
            rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            rgb_image_msg.header.stamp = self.get_clock().now().to_msg()

            # 创建ROS 2 Image消息并填充深度图像数据
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_image_msg.header.stamp = self.get_clock().now().to_msg()

            # zhuizonglujing
            shape = Twc.shape
            num_matrices, num_rows, num_columns = shape
            # print(num_matrices)  
            # print(num_rows)  
            # 遍历3D数组
            # for i in range(num_matrices):
            T = Twc[self.frame_count]
            # T = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]) 
            # for j in range(num_rows):
            # for k in range(num_columns):
            # 访问数组元素
            # element = self.Twc[i, j, k]
            print(T.flatten().tolist())
            print(type(T))
            print(type(T.flatten().tolist()))
            T_msg = Float32MultiArray()
            T_msg.layout.data_offset = 0
            T_msg.data = T.flatten().tolist()
            print(T_msg)
            # T_msg.header.stamp = self.get_clock().now().to_msg()


            # 在这里执行您想要的操作
            # 创建CameraInfo消息并填充相机参数
            # camera_info_msg = CameraInfo()
            # camera_info_msg.header.stamp = self.get_clock().now().to_msg()
            # camera_info_msg.width = rgb_image.shape[1]
            # camera_info_msg.height = rgb_image.shape[0]

            # 发布RGB图像、深度图像和相机信息
            self.publisher_rgb.publish(rgb_image_msg)
            self.publisher_depth.publish(depth_image_msg)
            self.publisher_T.publish(T_msg)
            # self.publisher_camera_info.publish(camera_info_msg)

            self.frame_count += 1  # 更新帧计数

            self.get_logger().info("成功发送RGB-D图像帧")

def main(args=None):
    rclpy.init(args=args)
    dataset_path = '/home/yang/office_0/imap/00'
    node = ReplicaRGBDPublisher(dataset_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
