Change Log:

Modified a little on the launch files.

Previously, the project_init.launch will bring up everything, but the launch order is messed up since we cannot set the time delay between two launch blocks. If the node starts right after the slam launch is called, strange errors will occur.

To address that, timed_roslaunch was introducted.

You need to sudo apt-get install ros-melodic-timed-roslaunch

By doing this, we are allowed to launch seperate launch files sequentially. And I put all the nodes in controller.launch.
