# junbot_core

# TODO List:

- [x] Publish point cloud to ROS
- [ ] Explore enviroment
      
   - [x] Yolov5 TensorRT support
   - [x] ~~OctoMap support~~
   - [x] Turning object database
      - [x] Segment 3D bounding box [link](https://github.com/lacie-life/SemanticMapping)
   	    - ~~[ ] Option 1: Segment point cloud by PointNet++ => get 3D bounding box of object (office enviroment) [[Ref](https://github.com/sc19aas/3D-object-detection)]~~
 		    - [x] Option 2: detect 3d cuboid [[Ref](https://github.com/aibo-kit/new_3dbbox_generation_method.git)] [[Ref](https://wym.netlify.app/2019-02-22-cubeslam/)]
   	    - ~~[ ] Darknet-ros-3d + ObjectDatabase test [[Link](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d)]~~
   	   
   	 - [x] Object filter (sometime crashed)
   	 - [x] Object Map
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
      - [x] Create an new layer for optimize coner
      - [ ] Algorithm
        - Input: 
            - [x] Sub Map topic => global costmap 
            - [x] Sub object in object layer from orb (temp: init object layer)
            - [x] Sub trajectory in path planner
        - Processing:
          - Custom theshold for coner 
        - Output:
          - List coner need to change in coner layer => update costmap => update path planner
   
 - [ ] Evaluate results (MavelMind)
   - [x] <b> Trajectory collection by MavelMind </b>
   - [ ] Improve GUI
   - [ ] Re-path planning

- [ ] Hardware
   - [x] Assembly
   - [ ] System calibration
      - [ ] Multi-odometry fusion [link](https://github.com/lacie-life/junbot_odometry_fusion)
      - [x] Robot TF

# Overview

<table>
  <tr>
    <td> <img src="./docs/front.jpg"  alt="1" width = 360px height = 640px ></td>

    <td><img src="./docs/side.jpg" alt="2" width = 360px height = 640px></td>
   </tr> 
</table>

# Install 

Follow this [link](https://github.com/lacie-life/junbot_core/blob/main/docs/install.md)

# Related Project

- [SemanticMapping](https://github.com/lacie-life/SemanticMapping)
- [junbot_relocalization](https://github.com/lacie-life/junbot_localization)
- [junbot_odometry_fusion](https://github.com/lacie-life/junbot_odometry_fusion)
- [junbot_mobile_app](https://github.com/lacie-life/junbot_app)


# Acknowledgement

- [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)


 
 
 
