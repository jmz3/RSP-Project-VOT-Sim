<h1 align="center">RSP Final Project</h1>
<h2 align="center">Vision Based Object Tracking Turtlebot</h2>

# 0. Content

# 1.  Packages

### Auto_move 
This package contains the source code for autopiloting the turtlebot3. The “auto_move_node” will publish velocity data to “/cmd_vel”. It will also use LiDar information from “/scan” from the robot to make turns and avoid obstacles. It offers an obstacle avoidance technique using a lidar range-finder. Turtlebot can detect obstacles in front and 15 degrees to the left and right, and robot will turn 0.5 once it reaches 0.8 meters away from obstacles. 
### Find_circle 
This package detects and calculates the position of a red ball in the world frame. It creates a node “find_center_node”, which subscribes to the topic “/raspicam_node/image” and publishes a geometry_msgs::Point message of the ball x, y coordinate to the topic “/ball_position”. For the object detection part, median blur and guassian blur are implemented for noise reduction, and hough gradient transform is used for locating the ball. A TF transformation calculation is performed to transfer pixel coordinates of the ball center to world coordinates, which is published as the goal for navigation.  
 
### My_tbot3_nav  
This package contains all the source code and launch file for the navigation section of our project. It includes nodes for simulatively initializing navigation process, publishing navigation points, and transcribing these points into the proper navigation goal messages. This package also contains the URDF for a ball to be placed in the environment. The launch files will launch and load the environment, run all the associated nodes, and load in the map and the ball URDF.   
 
### My_tbot3_sim 
This package contains the launch files for SLAM. It launches Gazebo and RVIZ and loads the robot. Based on the “control_mode” argument, it will also start the “auto_move_node” for autonomous SLAM or “teleop_twist_keyboard” for user-controlled SLAM. 
   
### My_tbot3_workcontrol 
This package contains a dynamic reconfigure file that offers a user interface for terminating the SLAM. The launch file will not only launch the SLAM files in “my_tbot3_sim” package but will also start the server waiting to end SLAM at the user’s request, after which map from SLAM will be saved.  
 
 
### Turtlebot3  
This folder contains all the necessary packages for Turtlebot3, including configuration files, URDF, and launch files.  

# 2. Functional Nodes & Topics
## “auto_move_node” 
Drive the robot by publishing geometry_msgs/Twist messages to /cmd_vel. It also subscribes to /scan and uses the LiDar information from the robot to avoid obstacles.  
 
### “controller_node” 
Node for dynamic reconfigure. It offers the user a button for ending the SLAM phase. The program can then initialize a shutdown sequence and save the map from SLAM. 
 
### “fake_goal_node” 
Publish waypoints’ coordinates in geometry_msgs/Point messaage to /fake_goal. It also subscribes to /move_base/result to collect robot’s result. 
 
### “nav_simulation_init_node” 
Publishing a geometry_msgs/PoseWithCovarianceStamped message to /initialpose to initialize robot’s position in the environment. It then subscribes to /odom and /fake_goal to transcribe the waypoint information from geometry_msgs/Point to geometry_msgs/PoseStamped and publishes to “/move_base_simple/goal”. 

# 3. Launch Files
