

## To compile this project:

catkin_make

## Commands to run Homework 01:

source devel/setup.bash
roslaunch ros_robotics ddrobot_rviz.launch model:=dd_robot.urdf gui:=true

urdf_to_graphiz src/ros_robotics/urdf/dd_robot.urdf 
evince dd_robot.pdf

roslaunch ros_robotics ddrobot_gazebo.launch 



