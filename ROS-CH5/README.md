# Programming with ROS

## Creating a workspace and package

### Creating workspace

`mdkir -p ~/catkin_ws/src`

`cd catkin_ws/src`

`cd catkin_init_workspace`

`cd ..`

`catkin_make`

* Change to `.bashrc` and add `source ~/catkin_ws/devel/setup.bash`

### Creating package

* Change to `src` folder

`catkin_create_pkg hello_world roscpp rospy std_msgs`
