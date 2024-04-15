# Pose Estimation for ROS 2

## Docker
Build docker image:
```
docker build -t pose_estimation .
```

Run image using GPU as
```
docker run --rm -ti --gpus all -v <assets>:/asset --network host pose_estimation
```

The `<assets>` directory shall contain the `config.yaml` (see example) and the point cloud asset in PCD format.

