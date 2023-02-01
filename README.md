# junbot_planner

# TODO List:

- [x] Create point cloud by ORB-SLAM3 (Pending release code)
- [x] Publish point cloud to ROS
- [ ] Explore enviroment
  ~~- [ ] Option 1: Segment point cloud by PointNet++ => get 3D bounding box of object (office enviroment) [[Ref](https://github.com/sc19aas/3D-object-detection)]~~
  ~~- [ ] Option 2: detect 3d cuboid [[Ref](https://github.com/aibo-kit/new_3dbbox_generation_method.git)] [[Ref](https://wym.netlify.app/2019-02-22-cubeslam/)]~~
  - [ ] Replace Yolo with Segmentation net => remove dynamic object (by interest class)
  - [ ] Turning object database
  - [ ] Add ZED example
- [ ] Re-path planning?
  - [ ] Plan of re-path planning
  - [ ] How to represent map with object for navigation stack ? (Grid map / OctoMap) ?
  - [ ] Planner? Which param need to change?
  - [ ] ....

# Install 

1. ROS Noetic

- vision_msgs 
  - sudo apt-get installros-noetic-vision-msgs
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping)
- [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins)

2. orb_slam3_ros

- CUDA 11.6 [[Link](https://developer.nvidia.com/cuda-11-6-0-download-archive)]
  - <i> Choose option and follow instructions </i>

- OpenCV 4.2 (with cuda support) [[Link](https://github.com/lacie-life/codecpp/blob/main/opencv_cuda.sh)]
  - <i>Note: Change CUDA_ARCH flag to your NVIDIA Device </i>

- PCL 1.8 [[Link](https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.0.zip)]
  - <i> Uncompress and build </i>

- Libtorch 1.12.1+cu116 (cxx11-abi) [[Link](https://download.pytorch.org/libtorch/cu116/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcu116.zip)]
  - <i> Unzip and change path to libtorch in  CMakeLists.txt</i>

- Pangolin

```
sudo apt-get install -y libglew-dev

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin 

./scripts/install_prerequisites.sh --dry-run recommended

mkdir build && cd build
cmake ..
cmake --build .

sudo make install
```

- TensorRT (Option)
- ZED SDK (Option)
- Realsense SDK (Option)

## Bug 

Pls refer links:

[PCL build Errors](https://blog.csdn.net/weixin_51925771/article/details/118405623)
 
[PointCloudMapping Errors](https://blog.csdn.net/hai_fellow_Z/article/details/123681382)

3. Note

- Make -j4
- If log too long, use can use this command:

```
catkin build -j4 &> log.txt
```


 
 
 
