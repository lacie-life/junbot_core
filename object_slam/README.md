Based on [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros)

# Requierments

- Ubuntu 20.04
- PCL 1.8
- OpenCV 4.5.2 (With CUDA Build)
- Pangolin
- Boost
- CUDA 11.6
- Libtorch 1.12.1+cu116 (cxx11-abi) (Option)
- TensorRT
- ZED SDK (Option)
- Realsense SDK (Option)

# Ref

https://blog.csdn.net/weixin_51925771/article/details/118405623
 
https://blog.csdn.net/hai_fellow_Z/article/details/123681382

# Note

```
# ZED 2 run
roslaunch zed_wrapper zed2.launch
# Object SLAM run
roslaunch object_slam slam_rgbd_zed.launch

# D435i run
roslaunch object_slam realsense_d435i.launch
# Object SLAM run
roslaunch object_slam d435i_rgbd_slam.launch
```
