"""Hardware-free native pipeline regression test; run with ROS_DOMAIN_ID=89."""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from livox_interfaces.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from dist_msg.msg import Dist


def test_native_full_points_and_debug_independence():
    rclpy.init()
    node = rclpy.create_node('native_test')
    processes = []
    distances, images, stamped = [], [], []
    prefix = Path(get_package_prefix('hik_livox_system')) / 'lib/hik_livox_system'
    try:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'k.txt').write_text('intrinsic\n100 0 50\n0 100 50\n0 0 1\n')
            (root/'t.txt').write_text('extrinsic\n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n')
            for executable, params in [
                ('stamper_cpp', ['input_topic:=/test/cloud', 'output_topic:=/test/stamped']),
                ('projector_cpp', [f'intrinsic_path:={root}/k.txt', f'extrinsic_path:={root}/t.txt',
                    'camera_topic:=/test/image', 'detection_topic:=/test/detection',
                    'lidar_topic:=/test/stamped', 'image_width:=100', 'image_height:=100',
                    'bbox_size:=1.0', 'debug_max_points:=1', 'debug_scale:=0.5',
                    'max_data_age:=0.5', 'debug:=true']),
            ]:
                command = [str(prefix/executable), '--ros-args']
                for value in params: command += ['-p', value]
                command += ['-r', '/distances:=/test/distances', '-r', '/debug_image:=/test/debug']
                processes.append(subprocess.Popen(command, stdout=subprocess.DEVNULL))
            pub_cloud=node.create_publisher(CustomMsg,'/test/cloud',10)
            pub_detection=node.create_publisher(Detection2DArray,'/test/detection',10)
            pub_image=node.create_publisher(Image,'/test/image',10)
            subs=[node.create_subscription(Dist,'/test/distances',distances.append,10),
                  node.create_subscription(Image,'/test/debug',images.append,10),
                  node.create_subscription(CustomMsg,'/test/stamped',stamped.append,10)]
            def spin_for(duration):
                end=time.monotonic()+duration
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            def send(points, stale=False):
                stamp=node.get_clock().now().to_msg()
                image=Image(height=100,width=100,encoding='rgb8',step=300,data=[0]*30000)
                image.header.stamp=stamp; pub_image.publish(image)
                detection=Detection2DArray(); detection.header.stamp=stamp
                if stale:detection.header.stamp.sec-=5
                box=Detection2D(); box.bbox.center.position.x=50.;box.bbox.center.position.y=50.
                box.bbox.size_x=10.;box.bbox.size_y=10.
                hypothesis=ObjectHypothesisWithPose();hypothesis.hypothesis.class_id='armor'
                box.results.append(hypothesis);detection.detections.append(box)
                pub_detection.publish(detection);spin_for(.04)
                cloud=CustomMsg();cloud.point_num=len(points)
                cloud.timebase=123456789
                cloud.points=[CustomPoint(x=float(x),y=float(y),z=float(z)) for x,y,z in points]
                pub_cloud.publish(cloud);spin_for(.2)
            deadline=time.monotonic()+10
            while pub_cloud.get_subscription_count()==0 and time.monotonic()<deadline:spin_for(.05)
            spin_for(1)
            send([(0,0,5),(.1,0,5),(-.1,0,5),(0,0,-1),(3,0,1)])
            assert distances and images and stamped
            assert stamped[-1].timebase==123456789 and len(stamped[-1].points)==5
            assert stamped[-1].header.stamp.sec>0
            assert distances[-1].count==1
            assert distances[-1].distances[0]==pytest.approx(math.sqrt(25.01),rel=1e-6)
            assert (images[-1].width,images[-1].height)==(50,50)
            data=images[-1].data
            assert sum(any(data[i:i+3]) for i in range(0,len(data),3))==1
            # A different second frame must not include any points from the first frame.
            send([(0,0,9)])
            assert distances[-1].distances[0]==pytest.approx(9)
            # Even-number median and invalid/zero-depth points.
            send([(0,0,5),(0,0,9),(float('nan'),0,2),(0,0,0)])
            assert distances[-1].distances[0]==pytest.approx(7)
            send([])
            assert math.isnan(distances[-1].distances[0])
            # Stale detections invalidate distance boxes but never stop the image stream.
            before=len(images)
            send([(0,0,7)],stale=True)
            assert distances[-1].count==0
            assert len(images)>before
            for process in processes: assert process.poll() is None
    finally:
        for process in processes:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
