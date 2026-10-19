import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from livox_interfaces.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from dist_msg.msg import Dist


class ProjectionNode(Node):
    """Project one Livox frame at a time and calculate box distances."""

    def __init__(self):
        super().__init__('projection_node')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('detection_topic', '/armor_detections')
        self.declare_parameter('lidar_topic', '/livox/stamped')
        self.declare_parameter('intrinsic_path', '/home/szx/works/hik_livox_workspace/src/livox_pcd_projection/config/intrinsic.txt')
        self.declare_parameter('extrinsic_path', '/home/szx/works/hik_livox_workspace/src/livox_pcd_projection/config/extrinsic.txt')
        self.declare_parameter('lidar_threshold', 20000)  # kept for config compatibility
        self.declare_parameter('refresh_rate', 10)
        self.declare_parameter('debug', False)
        self.declare_parameter('bbox_size', 0.8)
        self.declare_parameter('debug_max_points', 10000)
        self.declare_parameter('max_data_age', 0.2)
        self.declare_parameter('image_width', 1440)
        self.declare_parameter('image_height', 1080)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.intrinsic_path = self.get_parameter('intrinsic_path').value
        self.extrinsic_path = self.get_parameter('extrinsic_path').value
        self.refresh_rate = float(self.get_parameter('refresh_rate').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.bbox_size = float(self.get_parameter('bbox_size').value)
        self.debug_max_points = int(self.get_parameter('debug_max_points').value)
        self.max_data_age = float(self.get_parameter('max_data_age').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detection = None
        self.last_age_warning = self.get_clock().now()

        self.load_extrinsic(self.extrinsic_path)
        self.load_intrinsic_distortion(self.intrinsic_path)

        self.publisher = self.create_publisher(Dist, '/distances', 10)

        if self.debug:
            # Livox drives processing. Cached image/detections avoid a blocked three-way synchronizer.
            self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
            self.detection_sub = self.create_subscription(
                Detection2DArray, self.detection_topic, self.detection_callback, 10)
            self.lidar_sub = self.create_subscription(CustomMsg, self.lidar_topic, self.lidar_callback, 10)
            self.debug_publisher = self.create_publisher(Image, '/debug_image', 10)
            self.get_logger().info('Single-frame debug processing is driven by Livox frames')
        else:
            self.detection_sub = message_filters.Subscriber(
                self, Detection2DArray, self.detection_topic)
            self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.detection_sub, self.lidar_sub], 10, 1.0)
            self.ts.registerCallback(self.callback)

    @staticmethod
    def stamp_ns(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def image_callback(self, message):
        self.latest_image = message

    def detection_callback(self, message):
        self.latest_detection = message

    def lidar_callback(self, lidar_msg):
        if self.latest_image is None or self.latest_detection is None:
            return
        image_age = abs(self.stamp_ns(lidar_msg) - self.stamp_ns(self.latest_image)) / 1e9
        detection_age = abs(self.stamp_ns(lidar_msg) - self.stamp_ns(self.latest_detection)) / 1e9
        age = max(image_age, detection_age)
        if age > self.max_data_age:
            now = self.get_clock().now()
            if (now - self.last_age_warning).nanoseconds > 5_000_000_000:
                self.get_logger().warn(f'Skipping Livox frame: latest image age is {age:.3f}s')
                self.last_age_warning = now
            return
        self.process_frame(lidar_msg, self.latest_detection, self.latest_image)

    def callback(self, detection_msg, lidar_msg):
        self.process_frame(lidar_msg, detection_msg, None)

    def process_frame(self, lidar_msg, detection_msg, camera_msg):
        start = time.perf_counter()
        xyz = self.points_to_array(lidar_msg)
        projected, valid = self.project_points(xyz)
        self.publish_distances(detection_msg, projected, valid)

        if camera_msg is not None:
            image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='rgb8')
            debug_image = self.make_debug_image(image, projected, valid)
            output = self.bridge.cv2_to_imgmsg(debug_image, encoding='rgb8')
            output.header = camera_msg.header
            self.debug_publisher.publish(output)

        elapsed = time.perf_counter() - start
        if elapsed > 0.1:
            self.get_logger().info(f'Processed {xyz.shape[0]} points in {elapsed * 1000:.1f} ms')

    @staticmethod
    def points_to_array(lidar_msg):
        count = min(int(lidar_msg.point_num), len(lidar_msg.points))
        if count <= 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(
            [(p.x, p.y, p.z) for p in lidar_msg.points[:count]],
            dtype=np.float32)

    def project_points(self, xyz):
        if xyz.size == 0:
            return np.empty((0, 4), dtype=np.float64), np.zeros(0, dtype=bool)

        camera_xyz = xyz.astype(np.float64) @ self.extrinsic[:3, :3].T
        camera_xyz += self.extrinsic[:3, 3]
        depth = camera_xyz[:, 2]
        valid = np.isfinite(camera_xyz).all(axis=1) & (depth > 1e-6)

        uvw = camera_xyz @ self.intrinsic.T
        u = uvw[:, 0] / depth
        v = uvw[:, 1] / depth
        valid &= np.isfinite(u) & np.isfinite(v)
        valid &= (u >= 0) & (u < self.image_width)
        valid &= (v >= 0) & (v < self.image_height)

        distance = np.linalg.norm(xyz.astype(np.float64), axis=1)
        return np.column_stack((distance, u, v, depth)), valid

    def publish_distances(self, detection_msg, projected, valid):
        result = Dist()
        us = projected[:, 1] if projected.size else np.empty(0)
        vs = projected[:, 2] if projected.size else np.empty(0)
        distances = projected[:, 0] if projected.size else np.empty(0)

        for detection in detection_msg.detections:
            if not detection.results:
                continue
            result.obj_classes.append(detection.results[0].hypothesis.class_id)
            box = detection.bbox
            cx, cy = box.center.position.x, box.center.position.y
            half_w = box.size_x * self.bbox_size / 2.0
            half_h = box.size_y * self.bbox_size / 2.0
            inside = valid & (us >= cx - half_w) & (us <= cx + half_w)
            inside &= (vs >= cy - half_h) & (vs <= cy + half_h)
            selected = distances[inside]
            result.distances.append(float(np.median(selected)) if selected.size else float('nan'))

        result.count = len(result.distances)
        result.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(result)

    def make_debug_image(self, image, projected, valid):
        if projected.size == 0:
            return image
        indices = np.flatnonzero(valid)
        if self.debug_max_points > 0 and indices.size > self.debug_max_points:
            step = int(np.ceil(indices.size / self.debug_max_points))
            indices = indices[::step]

        u = projected[indices, 1].astype(np.int32)
        v = projected[indices, 2].astype(np.int32)
        depth = np.clip(projected[indices, 0], 0.0, 60.0)
        color_input = ((depth / 60.0) * 255.0).astype(np.uint8).reshape(-1, 1)
        colors = cv2.applyColorMap(color_input, cv2.COLORMAP_JET)[:, 0, ::-1]
        output = image.copy()
        output[v, u] = colors
        return output

    def load_extrinsic(self, path):
        rows = []
        with open(path, 'r', encoding='utf-8') as file:
            for line in file:
                if line.strip() and not line.strip().lower().startswith('extrinsic'):
                    rows.append([float(value) for value in line.split()])
        self.extrinsic = np.asarray(rows, dtype=np.float64)
        if self.extrinsic.shape != (4, 4):
            raise ValueError(f'Expected a 4x4 extrinsic matrix, got {self.extrinsic.shape}')

    def load_intrinsic_distortion(self, path):
        with open(path, 'r', encoding='utf-8') as file:
            lines = [line.strip() for line in file if line.strip()]
        start = lines.index('intrinsic') + 1
        self.intrinsic = np.asarray(
            [[float(value) for value in lines[start + row].split()] for row in range(3)],
            dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
