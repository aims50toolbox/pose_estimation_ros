# AI Tool for Pose Estimation with ROS 2 support

Pose estimation for a predefined CAD model using RGBD images. The CAD model shall be provided in standard PCD format (for reference, see [here](https://pointclouds.org/documentation/tutorials/pcd_file_format.html)). The tool provides the following algorithms to estimate the pose:

 * Iterative closest Point algorithm supported by object detection and segmentation
 * ~~FoundationPose~~ (soon...)

## Getting started
The tool provides an implementation for ROS 2 and notebooks to overview and understand the basics of the algorithms. For evaluating the ROS based version, please jump to [ROS 2 Docker Image with GPU support](#ros-2-docker-image-with-gpu-support). For evaluating and getting familiar with the algorithm in Jupyter notebooks, see [notebooks documentation](doc/Notebooks.md). For best practices regarding pose estimation, check [best practices](doc/BestPractices.md).

## ROS 2 Docker Image with GPU support
For ease of evaluation, you can build a Docker image right out-of-the-box. For using Docker, ensure, that Docker is [installed](https://docs.docker.com/engine/install/).

For building the Docker image with tag `pose_estimation`, enter the repository, and run:
```
docker build -t pose_estimation .
```

After the image is built, run the image using the GPU as
```
docker run --rm -ti --gpus all -v <assets>:/asset --network host pose_estimation
```

The `<assets>` directory shall contain the configuration as a `config.yaml` file (see [example](config.yaml.example)) and the point cloud asset in [PCD](https://pointclouds.org/documentation/tutorials/pcd_file_format.html) format, referenced by the configuration file.


### Docker trick
Please note, that ROS2 communcation protocol DDS uses shared memory when nodes are executed on the same machine. To avoid this, we disable shared memory feature (see `docker_trick.xml`).


## Topics

Subscriptions (to change, please see [configuration](config.yaml.example)):
 - `/camera/color/image_raw`
 - `/camera/aligned_depth_to_color/image_raw`
 - `/camera/aligned_depth_to_color/camera_info`

Publishers:
 - `/pose_estimation/diag/image`: image for diagnostics (`sensor_msgs/Image`)
 - `/pose_estimation/diag/points`: point cloud for diagnostics (`sensor_msgs/PointCloud2`)
 - `/pose_estimation/pose`: pose list in user defined format (`pe_interface/msg/PoseList.msg`), object2camera transformation (see `src/pe_interface`)

If the `diag` is enabled in the configuration, the diagnostic topics will receive diagnostic images and point clouds after each successful detection. Please note, that the point cloud contains the original point cloud restored from the depth image and the fitted reference object (coloured with red). **Note: using diagnostic messages *considerably* increases running time.**