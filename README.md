# RRTBase / RRTStar / RRTConnect Path Planner

## Overview
This repository contains a ROS 2-based path planning system using the RRTBase / RRTStar / RRTConnect algorithm. It includes nodes for map publishing, Basic RRT, RRTStar, RRTConnect, and a pose_publisher for start and goal locations.

**Components**:  
1. **map_publisher**: Publishes a map with obstacles.  
2. **rrt_planner_node**: This node generates the path according to a basic implementation of the RRT path planner.  
3. **rrt_connect_planner_node**: This node generates the path according to the RRT-Connect algorithm.  
4. **rrt_star_planner_node**: This node generates the path according to the RRT* algorithm.  
5. **pose_publisher**: This node publishes the start and goal locations for the RRT nodes to generate the path.  
6. **RViz**: A visualization tool for displaying the map and the generated paths.

## ROS 2 File Structure

```
├── images
│   ├── pose_publisher.png
│   ├── rrt_planner_and_map_publisher.png
│   ├── rrt_planner_lifecycle_manager.png
│   └── rviz2.gif
├── README.md
├── rrt_planner
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── lifecycle_bond_utils
│   │   │   └── lifecycle_bond_utils.hpp
│   │   └── rrt_planner
│   │       ├── common.hpp
│   │       ├── rrt_connect_planner_node.hpp
│   │       ├── rrt_planner_node.hpp
│   │       └── rrt_star_planner_node.hpp
│   ├── launch
│   │   ├── rrt_planner_and_map_publisher.launch.py
│   │   └── rrt_planner_lifecycle_manager.launch.py
│   ├── package.xml
│   └── src
│       ├── lifecycle_bond_utils
│       │   └── lifecycle_bond_utils.cpp
│       └── rrt_planner
│           ├── map_publisher.cpp
│           ├── pose_publisher.cpp
│           ├── rrt_connect_planner_node.cpp
│           ├── rrt_planner_node.cpp
│           └── rrt_star_planner_node.cpp
└── rrt_planner.rviz

```

## Prerequisites
1. Ubuntu 22.04
2. Ros2 Humble

## Setup

#### Open a 4 terminal 

**1. In terminal A**
  ```
  colcon build
  source install/setup.bash
  ros2 launch rrt_planner rrt_planner_and_map_publisher.launch.py
  ```
![Map Publisher and RRT Planner](/images/rrt_planner_and_map_publisher.png)

**2. In terminal B**
  ```
  source install/setup.bash
  ros2 launch rrt_planner rrt_planner_lifecycle_manager.launch.py
  ```
![Lifecycle Manager](/images/rrt_planner_lifecycle_manager.png)

**3. In terminal C**
  ```
  source install/setup.bash
  ros2 run rrt_planner pose_publisher
  ```
![Pose Publisher](/images/pose_publisher.png)

**4. In terminal D**
  ```
  source install/setup.bash
  rviz2
```
> **Note:** Rviz should use rrt_planner.rviz file configuration.

## Output
1. All RRT planner variants will generate a path depending on the start and goal locations provided by the pose_publisher, as long as the provided start and goal locations are valid.

2. The generated path can be visualized in RViz.  
 <img src="/images/rviz2.gif" width="500" />

#### Developer Information

- **Name:** Shailesh Pawar  
- **Contact:** shaileshpawar320@gmail.com
