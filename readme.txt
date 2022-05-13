To successfully run the simulation for our project, please first add the following line to your ~/.bashrc file: 

export TURTLEBOT3_MODEL=waffle_pi

This will be used to identify the correct model of robot that we use in our simulation. 

There are two other packages that you might need to install, which are gmapping package for SLAM, and Timed-Roslaunch package.

If you don't already have gmapping, you can clone it into your workspace by 

$ git clone https://github.com/ros-perception/slam_gmapping.git

As for the timed-roslaunch, you can install it with

$ sudo apt-get install ros-${ROSDistro}-timed-roslaunch

After acquiring the packages above (and of course our package itself), the entire workspace can then be built with "catkin build".  

The first section of our simulation is done using the existing slam packages for Turtlebot3. There are 

The initialization launch file is under the my_tbot3_workcontrol package: project_init.launch.

This launch file will start
1. The Turtlebot3 SLAM package
2. A gazebo environment (namely turtlebot3_world.launch)
3. Teleop_twist_keyboard for driving the robot
4. A dynamic reconfigure interface for the user to terminate the SLAM process 
5. Load a yaml config into the dynamic reconfigure

When the user chooses to terminate the SLAM process, the user can select YES for Switch_To_Navigation. 

The program will automatically save the map and kill SLAM nodes.  (controller.cpp under my_tbot3_workcontrol package).

Then for Navigation, launch start_nav_with_gz.launch under my_tbot3_nav package. 

An initial pose will be set for the robot. 

To set navigation goals, either click on the "2D Nav Goal" and draw an arrow in RVIZ 
	Or publish to /move_base_simple/goal with msg type: geometry_msgs/PoseStamped and frame_id: map


