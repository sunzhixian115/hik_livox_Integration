import math

import numpy as np
from builtin_interfaces.msg import Time
from livox_interfaces.msg import CustomMsg, CustomPoint
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from livox_pcd_projection.projectCloud import ProjectionNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    class _Now:
        nanoseconds = 0

        @staticmethod
        def to_msg():
            return Time()

    def now(self):
        return self._Now()


def make_node():
    node = ProjectionNode.__new__(ProjectionNode)
    node.extrinsic = np.eye(4, dtype=np.float64)
    node.intrinsic = np.array(
        [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    node.image_width = 100
    node.image_height = 100
    node.bbox_size = 1.0
    node.debug_max_points = 1
    node.publisher = _Publisher()
    node.get_clock = lambda: _Clock()
    return node


def make_cloud(points):
    message = CustomMsg()
    message.point_num = len(points)
    message.points = [CustomPoint(x=x, y=y, z=z) for x, y, z in points]
    return message


def make_detection():
    detection = Detection2D()
    detection.bbox.center.position.x = 50.0
    detection.bbox.center.position.y = 50.0
    detection.bbox.size_x = 10.0
    detection.bbox.size_y = 10.0
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'armor'
    detection.results.append(hypothesis)
    array = Detection2DArray()
    array.detections.append(detection)
    return array


def test_single_frame_projects_all_points_and_filters_invalid_points():
    node = make_node()
    cloud = make_cloud([
        (0.0, 0.0, 5.0),       # inside box
        (0.1, 0.0, 5.0),       # inside box
        (-0.1, 0.0, 5.0),      # inside box
        (0.0, 0.1, 5.0),       # inside box
        (0.0, 0.0, -1.0),      # behind camera
        (3.0, 0.0, 1.0),       # outside image
    ])

    xyz = node.points_to_array(cloud)
    projected, valid = node.project_points(xyz)
    node.publish_distances(make_detection(), projected, valid)

    result = node.publisher.messages[-1]
    assert xyz.shape == (6, 3)
    assert valid.tolist() == [True, True, True, True, False, False]
    assert result.count == 1
    assert result.obj_classes == ['armor']
    assert math.isclose(result.distances[0], math.sqrt(25.01), rel_tol=1e-6)


def test_debug_sampling_does_not_change_distance_inputs():
    node = make_node()
    cloud = make_cloud([(0.0, 0.0, 5.0), (0.1, 0.0, 5.0), (-0.1, 0.0, 5.0)])
    projected, valid = node.project_points(node.points_to_array(cloud))
    detection = make_detection()

    node.publish_distances(detection, projected, valid)
    distance_before = node.publisher.messages[-1].distances[0]

    image = np.zeros((100, 100, 3), dtype=np.uint8)
    debug_image = node.make_debug_image(image, projected, valid)
    node.publish_distances(detection, projected, valid)
    distance_after = node.publisher.messages[-1].distances[0]

    assert debug_image.shape == image.shape
    assert math.isclose(distance_before, distance_after, rel_tol=1e-12)
    assert np.count_nonzero(debug_image) <= 3 * node.debug_max_points
