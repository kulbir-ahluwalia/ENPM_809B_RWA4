## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
copy group4_rwa4 package here
cd ..
catkin_make
```

## Running the code


Instructions for running package:
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch group4_rwa4 group4_rwa4.launch


#Start moveit for arm1 (https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/moveit_interface)
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1


#Start node
rosrun group4_rwa4 main_node
