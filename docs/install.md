# Install 

1. ROS Noetic

- vision_msgs 

```
sudo apt-get install ros-noetic-vision-msgs
```

2. Support packages 

```
sudo apt install qtmultimedia5-dev

sudo apt-get install ros-noetic-ddynamic-reconfigure

sudo apt-get -y install libflann-dev

sudo apt-get -y install libflann1.9

sudo apt install build-essential libboost-system-dev libboost-thread-dev libboost-program-options-dev libboost-test-dev

sudo apt-get install cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev

sudo apt-get install libvtk7-dev

sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

```

3. object_mapping (Skip CUDA Install if use jetpack in Jetson series)

- CUDA 11.6 [[Link](https://developer.nvidia.com/cuda-11-6-0-download-archive)]
  - <i> Choose option and follow instructions </i>

- OpenCV 4.5.2 (With CUDA Build) [[Link](https://github.com/lacie-life/codecpp/blob/main/opencv_cuda.sh)]
  - <i>Note: Change CUDA_ARCH flag to your NVIDIA Device and OpenCV version</i>
  - <i>Note: If you have conflict with Ros opencv, remove them and install ros depends manual </i>

- PCL 1.8 [[Link](https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.0.zip)]
  - <i> Uncompress and build </i>

- TensorRT
- ZED SDK
- Realsense SDK (2.50.0)
- Realsense ROS (2.3.1)

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