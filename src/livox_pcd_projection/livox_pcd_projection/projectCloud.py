import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
    ObjectHypothesis
)
import cv2
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from livox_interfaces.msg import CustomMsg
from dist_msg.msg import Dist
import numpy as np
import time

try:
    import cupy as _cupy
    _cupy.cuda.runtime.getDeviceCount()
    cp = _cupy
except Exception:
    cp = np


def to_numpy(value):
    if cp is np:
        return np.asarray(value)
    return cp.asnumpy(value)

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('detection_topic', '/armor_detections')
        self.declare_parameter('lidar_topic', '/livox/stamped')
        self.declare_parameter('intrinsic_path', '/home/szx/works/hik_livox_workspace/src/livox_pcd_projection/config/intrinsic.txt')
        self.declare_parameter('extrinsic_path', '/home/szx/works/hik_livox_workspace/src/livox_pcd_projection/config/extrinsic.txt')
        self.declare_parameter('lidar_threshold', 20000)
        self.declare_parameter('refresh_rate', 30)
        self.declare_parameter('debug', False)
        self.declare_parameter('bbox_size', 0.8)

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.intrinsic_path = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        self.extrinsic_path = self.get_parameter('extrinsic_path').get_parameter_value().string_value
        self.lidar_threshold = self.get_parameter('lidar_threshold').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.bbox_size = self.get_parameter('bbox_size').get_parameter_value().double_value
        
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Detection topic: {self.detection_topic}")
        self.get_logger().info(f"Lidar topic: {self.lidar_topic}")
        self.get_logger().info(f"intrinsic file path: {self.intrinsic_path}")
        self.get_logger().info(f"extrinsic file path: {self.extrinsic_path}")
        self.get_logger().info(f"lidar threshold: {self.lidar_threshold}")
        self.get_logger().info(f"node refresh rate: {self.refresh_rate}")
        self.get_logger().info(f"debug mode: {self.debug}")
        self.get_logger().info("Finished parsing parameters")

        self.load_extrinsic(self.extrinsic_path)
        self.load_intrinsic_distortion(self.intrinsic_path)

        self.lidar_decay_list = [] #存储最近的lidar点云

        self.get_logger().info("Starting message filter subscribers and registering callbacks")
        
        # 创建发布者，用于发布距离消息
        self.publisher = self.create_publisher(Dist, '/distances', 10)

        # 创建订阅者，用于订阅检测消息和点云消息
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
        self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)

        if self.debug:
            # 创建订阅者，用于订阅相机图像消息
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub, self.lidar_sub], 10, 0.1)#创建时间同步器，用于同步相机图像、检测消息和点云消息
            self.ts.registerCallback(self.debug_callback)#注册回调函数，用于处理同步后的消息
            self.debug_publisher = self.create_publisher(Image, '/debug_image', 10)#创建发布者，用于发布调试图像消息
        else :
            self.ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.lidar_sub], 10, 1)#创建时间同步器，用于同步检测消息和点云消息
            self.ts.registerCallback(self.callback)#注册回调函数，用于处理同步后的消息

    def debug_callback(self, camera_msg, detection_msg, lidar_msg): #处理调试模式下的回调函数
        projected_points = np.array([]) #存储投影后的点云
        bridge = CvBridge() #创建OpenCV桥接器
        pts_num = lidar_msg.point_num #获取点云点数
        self.get_logger().info(f"Received {pts_num} points from lidar") #打印点云点数
        cv_image = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='rgb8') #将相机图像消息转换为OpenCV图像
        if len(self.lidar_decay_list) > self.lidar_threshold / pts_num + 1: #如果点云数量超过阈值，删除最早的点云
           self.lidar_decay_list.pop(0) #删除最早的点云
        self.lidar_decay_list.append(lidar_msg) #将当前点云添加到列表中
        x, y, z = [], [], []
        for i in range(len(self.lidar_decay_list)):
            for j in range(self.lidar_decay_list[i].point_num):
                x.append(self.lidar_decay_list[i].points[j].x)
                y.append(self.lidar_decay_list[i].points[j].y)
                z.append(self.lidar_decay_list[i].points[j].z)

            x_temp, y_temp, z_temp = np.array(x)[:,None], np.array(y)[:,None], np.array(z)[:,None]
            ones = np.ones((x_temp.shape[0], 1))
            points = cp.asarray(np.hstack((x_temp, y_temp, z_temp, ones)))
            if projected_points.size == 0: #如果投影后的点云为空
                projected_points = self.getTheoreticalUV(x, points)
            else:
                projected_points = np.hstack((projected_points, self.getTheoreticalUV(x, points))) #将当前点云投影到图像上
            
        image = self.depth_to_color(projected_points, cv_image)  # 将投影后的点云转换为颜色图像
        img_msg = bridge.cv2_to_imgmsg(image, encoding='rgb8') #将颜色图像转换为ROS图像消息
        img_msg.header.stamp = camera_msg.header.stamp #设置图像消息的时间戳为相机图像的时间戳
        self.debug_publisher.publish(img_msg) #发布调试图像消息

    
#@param projected_points: 投影后的点云
#@param image: 输入图像
#@param max_depth: 最大深度
#@param min_depth: 最小深度
#@return: 颜色图像
    def depth_to_color(self, projected_points, image, max_depth = 60, min_depth = 3): #将投影后的点云转换为颜色图像
        scale = (max_depth - min_depth) / 10 #计算深度缩放因子
        depth = projected_points[0, :] #获取深度通道
        #颜色分配：1.深度小于最小深度，颜色为蓝色 2.深度在最小深度加上缩放因子到最小深度加上2倍缩放因子，颜色为绿色 3.深度在最小深度加上2倍缩放因子到最小深度加上4倍缩放因子，颜色为黄色 4.深度在最小深度加上4倍缩放因子到最小深度加上7倍缩放因子，颜色为红色 5.深度在最小深度加上7倍缩放因子到最大深度加上10倍缩放因子，颜色为紫色 6.深度大于最大深度加上10倍缩放因子，颜色为白色
        for i in range(depth.shape[0]): #遍历每个点
            if depth[i] < min_depth:
                r, g, b = 0, 0, 0xff
            elif depth[i] < min_depth + scale:
                r = 0
                g = int((depth[i] - min_depth) / scale * 255) & 0xff
                b = 0xff
            elif depth[i] < min_depth + 2 * scale:
                r = 0
                g = 0xff
                b = (0xff - int((depth[i] - min_depth - scale) / scale * 255)) & 0xff
            elif depth[i] < min_depth + 4 * scale:
                r = int((depth[i] - min_depth - scale*2) / scale * 255) & 0xff
                g = 0xff
                b = 0
            elif depth[i] < min_depth + 7 * scale:
                r = 0xff
                g = (0xff - int((depth[i] - min_depth - scale*4) / scale * 255)) & 0xff
                b = 0
            elif depth[i] < max_depth + 10 * scale:
                r = 0xff
                g = 0
                b = int((depth[i] - min_depth - scale*7) / scale * 255) & 0xff
            else:
                r, g, b = 0xff, 0, 0xff

            bgr = [r, g, b] #将颜色转换为BGR格式
            u, v = projected_points[1, i], projected_points[2, i] #获取点的投影坐标
            image = cv2.circle(image, (int(u), int(v)), 1, bgr, -1)
        
        return image

    #@param detection_msg: 检测消息
    #@param lidar_msg: LIDAR消息

    def callback(self, detection_msg, lidar_msg): #回调函数 做了以下工作：1.将LIDAR点云投影到图像上 2.将投影后的点云转换为颜色图像 3.将颜色图像发布到调试话题 4.计算每个检测框的平均距离
        pts_num = lidar_msg.point_num #获取点云数量
        if len(self.lidar_decay_list) > self.lidar_threshold / pts_num + 1: #如果点云数量超过阈值，删除最早的点云
           self.lidar_decay_list.pop(0) #删除最早的点云
        self.lidar_decay_list.append(lidar_msg) #将当前点云添加到列表中

        # saving all the lidar points in to an array
        projected_points = np.array([])#初始化投影后的点云数组
        x, y, z = [], [], []
        for i in range(len(self.lidar_decay_list)):
            for j in range(self.lidar_decay_list[i].point_num):
                x.append(self.lidar_decay_list[i].points[j].x)
                y.append(self.lidar_decay_list[i].points[j].y)
                z.append(self.lidar_decay_list[i].points[j].z)

            x_temp, y_temp, z_temp = np.array(x)[:,None], np.array(y)[:,None], np.array(z)[:,None]
            ones = np.ones((x_temp.shape[0], 1))
            points = cp.asarray(np.hstack((x_temp, y_temp, z_temp, ones)))
            if projected_points.size == 0: #如果投影后的点云为空
                projected_points = self.getTheoreticalUV(x, points) #将当前点云投影到图像上
            else:
                projected_points = np.hstack((projected_points, self.getTheoreticalUV(x, points))) #将当前点云投影到图像上并拼接在投影后的点云数组中

            x.clear()
            y.clear()
            z.clear()

        # 计算每个检测框内雷达点的欧氏距离中值
        dist_msg = Dist()
        for detection in detection_msg.detections: #遍历每个检测框
             #解析检测消息中的数据
            dist_msg.obj_classes.append(detection.results[0].hypothesis.class_id) #添加检测框的类别
            bbox = detection.bbox #获取检测框的边界框
            pose2d = bbox.center #获取检测框的中心坐标
            center_x, center_y = pose2d.x, pose2d.y #获取检测框的中心坐标
            width, height = bbox.size_x, bbox.size_y #获取检测框的宽度和高度

            temp = []
            us = projected_points[1, :] #获取u通道
            vs = projected_points[2, :] #获取v通道
            # 计算检测框内的点!!!
            right = center_x + width / 2 * self.bbox_size
            left = center_x - width / 2 * self.bbox_size
            top = center_y - height / 2 * self.bbox_size
            bottom = center_y + height / 2 * self.bbox_size
            temp = projected_points[0, (us >= left) & (us <= right) & (vs >= top) & (vs <= bottom)] #获取检测框内的点

            temp = temp[~np.isnan(temp)]
            result = self.calc_dist(temp)
            dist_msg.distances.append(result)
        
        # publish Dist message
        dist_msg.header.stamp = self.get_clock().now().to_msg() # timestamp
        dist_msg.count = len(detection_msg.detections)
        self.publisher.publish(dist_msg)

    def calc_dist(self, dist_array):
        """Return the median Euclidean distance of points in a detection box."""
        return np.median(dist_array)
                                
    def load_extrinsic(self, extrinsic_path): #加载外参矩阵
        file = open(extrinsic_path, 'r')
        lines = file.readlines()
        extrinsic = np.zeros((4, 4))
        for i, line in enumerate(lines):
            if i == 0:
                continue
            else:
                line = line.split('  ')
                extrinsic[i-1]= [float(x) for x in line]
    
        file.close()
        self.extrin = cp.asarray(extrinsic[:3, :])
        self.extrinsic = cp.asarray(extrinsic)
        
        self.get_logger().info('Loaded extrinsic data')

    def load_intrinsic_distortion(self, intrinsic_path): #加载内参矩阵和畸变系数
        file = open(intrinsic_path, 'r')
        lines = file.readlines()
        intrinsic = np.zeros((3, 3))
        distortion = np.zeros((5, 1))
        skip = [0, 4, 5]
        for i, line in enumerate(lines):
            if i in skip:
                continue
            elif i < 4:
                line = line.split(' ')
                intrinsic[i-1]= [float(x) for x in line if x != '']
            else:
                line = line.split(' ')
                for j, dist in enumerate(line):
                    if dist != '':
                        distortion[j] = float(dist)
        self.intrinsic = cp.asarray(intrinsic)
        self.distortion = cp.asarray(distortion)
        file.close()
        self.get_logger().info('Loaded intrinsic data')

    def getTheoreticalUV(self, x, points): # 计算理论上的u,v坐标
        # intrinsic is 3x3
        # extrinsic is 3x4
        # m3 4x1
        result = cp.matmul(cp.matmul(self.intrinsic, self.extrin), points.T)#计算投影后的点云
        
        depth = result[2,:] #获取深度通道
        u = result[0,:] / depth #计算u通道
        v = result[1,:] / depth #计算v通道
        u, v = to_numpy(u), to_numpy(v) #将结果转换为numpy数组
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(to_numpy(points[:, 1]), dtype=np.float64)
        z = np.asarray(to_numpy(points[:, 2]), dtype=np.float64)
        distance = np.sqrt(x * x + y * y + z * z)

        return np.vstack((distance, u, v))


def main(args=None):
    rclpy.init(args=args)

    node = ProjectionNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
