# Native live projection pipeline

`bringup.launch.py` uses `stamper_cpp` and `projector_cpp` from this package.
The Python implementation remains available for comparison and regression tests.

- Every received Livox message is one independent frame; historical points are never accumulated.
- Stamper preserves all points, point offsets and `timebase`, and updates only the ROS header timestamp to receipt time. This is not hardware time synchronization.
- All finite points in front of the camera are projected. All points inside each detection box participate in the median Euclidean range, with no point sampling for distance calculation.
- Detection boxes retain the original `bbox_size=0.8` inset. The range is measured from the lidar origin, not the camera origin.
- A missing/older-than-0.5-second detection produces a `Dist` with `count=0`. A fresh box with no lidar returns produces a NaN distance. The distance header is the input cloud header.
- Debug images continue even when detection results are missing/stale. Images must be within 0.2 seconds of the cloud. The detection source image is preferred when fresh; otherwise the latest image is used.
- Debug defaults: 720x540, at most 5000 plotted points, target 10 Hz. Only the displayed points are sampled. A 10% scheduling tolerance accommodates input timestamp jitter.
- Latest detections may be reused between inference updates, so a 10 Hz distance topic does not imply 10 independent detection updates. Motion can still cause spatial misalignment; receipt-time stamping also includes sensor/transport delay.

The detector uses a one-image input queue and one OpenCV thread. Its `cpu_threads` parameter defaults to 4; 0 retains OpenVINO defaults. This maps to the optional `NNDEPLOYMENT_CPU_THREADS` backend override.

## Build and start

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hik_livox_system ros2_armor_detector --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch hik_livox_system bringup.launch.py
```

Native projector parameters include `debug`, `bbox_size`, `max_data_age`, `max_image_age`, `debug_max_points`, `debug_scale`, `debug_rate`, `image_width`, `image_height`, topics and calibration paths. `debug_max_points=0` draws all visible points. Calibration uses the existing pinhole projection without distortion correction, matching the previous implementation. Use the same intrinsics/image geometry as before.

## Hardware-free regressions

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=89 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/hik_livox_system/test/test_native_pipeline.py \
  src/livox_pcd_projection/test/test_projectCloud.py
```

The native regression starts actual compiled stamper/projector processes on isolated test topics, and verifies full-point median, frame independence, timestamp forwarding, empty cloud handling, debug downsampling and continued image output with stale detections.
