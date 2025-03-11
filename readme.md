# Bharat Forge MidPrep: Autonomous Mobile Robot Swarm for Dynamic Environments

This repository focuses on designing and implementing a swarm of autonomous mobile robots (AMRs) capable of navigating dynamic, GPS-denied environments with optimized path planning, efficient collaboration, and adaptability to environmental changes.

## System Setup
### **System Requirements**
- **Operating System**: Ubuntu 22.04 LTS
- **ROS2**: Humble
- **Simulation Tools**: Gazebo classic

### **Environment Specifications**
- **Swarm Size**: 4 or more turtlebot3 waffle robots
- **Robot Type**: Differential drive
- **Objects in Environment**: 10 or more unique objects
- **Dynamic Obstacles**: 3 or more turtlebot3 burger robots

## Installation Instructions
1. **Install Required Dependencies**
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox
   sudo apt install ros-humble-slam-toolbox
   sudo apt install ros-humble-tf2-tools
   sudo apt install ros-humble-rviz2
   sudo apt install python3-pip
   pip3 install numpy matplotlib opencv-python
   pip3 install ultralytics
   ```
 
2. **Setting up ROS2**
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   ```
  
3. **Setup ROS 2 Workspace**
   ```bash
   cd /bharatforge_ws
   colcon build
   source install/setup.bash
   ```

## Execution Steps

### **Running Autonomous Exploration**
1. Launch the simulation environment:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_ps6_world.launch.py
   ```
2. Launch ROS2 navigation stack:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
   ```
3. Launch ROS2 slam package:
   ```bash
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
   ```
4. Run RViz2 for visualization:
   ```bash
   ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```
5. Run the exploration package
   ```bash
   ros2 launch explore_lite explore.launch.py
   ```
### Clustering object coordinates
(Internally)A server is implemented which clusters multiple coordinates of same object occuring due to errors
```bash
ros2 run ps6_py_control_pkg k-means_server
```
```bash
ros2 service call /process_kmeans arm_interface_pkg/srv/Kmeans "{x_coordinates: [<list_of_x_coordinates>], y_coordinates: [<list_of_y_coordinates>]}"
```

### Idle space allocation
A server implementing OpenCV, takes map, exclusion radius and number of robots as request and responds with two lists of x and y coordinates of spaces available for robots to standby
```bash
ros2 run ps6_py_control_pkg available_spaces_server
```
(Optional)This server can manually be called via:
```bash
ros2 service call /select_points arm_interface_pkg/srv/SelectPoints "{image_path: '<path_to_map.pgm_as_string>', num_points: <number_of_robots_spawned>, exclusion_radius: <available_space_for_each_robot>}"
```
### Navigation
To start the navigation server, which allocates goal pose to specified a robot
```bash
ros2 run ps6_py_control_pkg navigation_server
```
(Optional)This server can manually be called via:
```bash
ros2 service call /navigate_robot arm_interface_pkg/srv/Navigator "{robot_namespace: <namespace_of_robot>, x_goal: <x_coordinate_of_goal>, y_goal: <y_coordinate_of_goal>, yaw_goal: <rotation_of_goal_pose>}"
```
Another server which takes request of coordinates of multiple position of a type of object and requests the navigation server with the robot allocated optimally(example multiple fire extinguishers being available):
```bash
ros2 run ps6_py_control_pkg obtimium_client
```
(Optional)This server takes lists of objects stored in the central server and can manually be called via:
```bash
ros2 service call /optimium_bot arm_interface_pkg/srv/TurtleOptPath "{label: "<object_type>"}"
```

## Implementations:

### Exploration and Labelling
- **Frontier Based Exploration:** is used by a single robot which autonomously creates the map of whole environment with Simultaneous Localization And Mapping algorithm.
- **YOLO:** labels each visible object and the camera matches its readings with lidar to obtain their positions.
- **K-means Clustering:** is used to avoid a single object being marked multiple times.
- Each robot uses Adaptive Monte Carlo Localization algorithm along with Extended Kalman filter to precisely localize the themselves on the global map created while exploration.

### Spawn allocation
 OpenCV is used to allocate idle points for robots to standby when no task is assigned, based on open spaces of a given exclusion radius on the map.

### Task allocation:
- LLM-Powered GUI: The GUI interprets user commands, such as requesting a robot to move near a previously marked object.
- Central brain then computes each objects' distance from each robot and allocates specific robot to each goal achieved through action servers in Nav2.

## Key Features
### Scalability
- The algorithm used for allocating idle points ensures efficient division of robots in the map even on increasing the number of robots and changing maps.

### Shared Knowledge
- The central server has coordinates of each robot and a global map, while each robot only has its local costmap for obstacle avoidance.
- Also the coordinates of objects is stored on the central server, which enables efficient task assignment of robots.

### Bonus Feature
- Integrated chatbot interface for natural language task assignment

## Demonstrations
Simulations showcasing:
1. Autonomous Exploration
2. Obstacle Avoidance
3. Task allocation

Simulation videos are available in the `/demos` folder.

For more details, refer to the [technical report](./docs/technical_report.pdf)
