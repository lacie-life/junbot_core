# junbot_planner

# TODO List:

- [x] Create point cloud by ORB-SLAM3 (Pending release code)
- [x] Publish point cloud to ROS
- [ ] System calibration
   - [ ] Multi camera fusion
   - [x] Robot TF
- [ ] Explore enviroment
   
   - [ ] ~~Replace Yolo with Segmentation net => remove dynamic object (by interest class)~~
   
   - [x] Yolov5 TensorRT support
   
   - [x] ~~OctoMap support~~
   
   - [ ] Turning object database
   	   - [x] Segment 3D bounding box
   	         - [ ] ~~Option 1: Segment point cloud by PointNet++ => get 3D bounding box of object (office enviroment) [[Ref](https://github.com/sc19aas/3D-object-detection)]~~
 		         - [x] Option 2: detect 3d cuboid [[Ref](https://github.com/aibo-kit/new_3dbbox_generation_method.git)] [[Ref](https://wym.netlify.app/2019-02-22-cubeslam/)]
   	   	   - [ ] ~~Darknet-ros-3d + ObjectDatabase test [[Link](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d)]~~
   	   
   	   - [x] Object filter (sometime crashed)
   	   - [ ] <b> Improve Object tracker </b>
   	   - [ ] <b> Improve Object filter </b>
   	   - [ ] Add object pose optimize ?
   	   - [ ] Object Map
   	   
   - [x] Add ZED example
   - [x] Add D455 example 

- [ ] Re-path planning?
   - [x] Plan of re-path planning
   - [x] How to represent map with object for navigation stack ? (Grid map / OctoMap) ? 
      - [x] CostMap layer? ([Ref](http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer))
      - [x] Node Add Object
      - [x] Get waypoint path
      - [x] Node subcribe global costmap 
   - [x] Planner? Which param need to change?
      - [x] CostMap param => add object zone 
      - ~~[x] Planner param (Only tuning)~~

   - Adaptive Costmap layer
      - [ ] Create an new layer for optimize coner
      - [ ] Algorithm
        - Input: 
            - [x] Sub Map topic => global costmap 
            - [ ] Sub object in object layer from orb (temp: init object layer)
            - [x] Sub trajectory in path planner
        - Processing:
          - Custom theshold for coner 
        - Output:
          - List coner need to change in coner layer => update costmap => update path planner
   
 - [ ] Evaluate results (MavelMind)
   - [ ] <b> Trajectory collection by MavelMind </b>
   - [ ] Improve GUI
   - [ ] SLAM
   - [ ] Re-path planning

# Install 

1. ROS Noetic

- vision_msgs 
  - sudo apt-get install ros-noetic-vision-msgs
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping)
- [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins)

2. Support packages 

```
sudo apt install qtmultimedia5-dev

sudo apt-get install ros-noetic-ddynamic-reconfigure
```

3. object_slam (Skip CUDA Install if use jetpack in Jetson series)

- CUDA 11.6 [[Link](https://developer.nvidia.com/cuda-11-6-0-download-archive)]
  - <i> Choose option and follow instructions </i>

- OpenCV 4.5.2 (With CUDA Build) [[Link](https://github.com/lacie-life/codecpp/blob/main/opencv_cuda.sh)]
  - <i>Note: Change CUDA_ARCH flag to your NVIDIA Device and OpenCV version</i>
  - <i>Note: If you have conflict with Ros opencv, remove them and install ros depends manual </i>

- PCL 1.8 [[Link](https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.0.zip)]
  - <i> Uncompress and build </i>

- Pangolin

```
sudo apt-get install -y libglew-dev

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin 

./scripts/install_prerequisites.sh --dry-run recommended

mkdir build && cd build
cmake .. or cmake .. -DPython_EXECUTABLE='/usr/bin/python3'
cmake --build .

sudo make install
```

- TensorRT
- ZED SDK (Option)
- Realsense SDK (Option)

3. TensorRT with Yolov5 model

3.1. Generate TensorRT model
```
git clone -b v7.0 https://github.com/ultralytics/yolov5.git
# create conda envs and install requierments.txt for running gen_wts.py
# stupid scripts

git clone -b yolov5-v7.0 https://github.com/wang-xinyu/tensorrtx.git
cd yolov5/
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
cp [PATH-TO-TENSORRTX]/yolov5/gen_wts.py .
python gen_wts.py -w yolov5s.pt -o yolov5s.wts
# A file 'yolov5s.wts' will be generated.

cd [PATH-TO-TENSORRTX]/yolov5/
# Update kNumClass in src/config.h if your model is trained on custom dataset
mkdir build
cd build
cp [PATH-TO-ultralytics-yolov5]/yolov5s.wts . 
cmake ..
make

# Generate engine file (engine include 80 class of coco dataset)
./yolov5_det -s yolov5s.wts yolov5s.engine s
```

3.2. Build Detector

```
cd junbot_planner/object_slam/object_slam/Thirdparty/yolov5
mkdir build
cd build
cmake ..
make
```

# Bug 

Pls refer links:

[PCL build Errors](https://blog.csdn.net/weixin_51925771/article/details/118405623)
 
[PointCloudMapping Errors](https://blog.csdn.net/hai_fellow_Z/article/details/123681382)

# Note

- Make -j4
- If log too long, use can use this command:

```
catkin build -j4 &> log.txt
```


 
 
 
