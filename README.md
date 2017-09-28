

## To compile this project:

catkin_make

## Commands to run Homework 01:

source devel/setup.bash
roslaunch ros_robotics ddrobot_rviz.launch model:=dd_robot.urdf gui:=true

urdf_to_graphiz src/ros_robotics/urdf/dd_robot.urdf 
evince dd_robot.pdf

roslaunch ros_robotics ddrobot_gazebo.launch 


## Commands to run Homework 02:

'source devel/setup.bash'
'rosrun beginner_tutorials talker'
'rosrun beginner_tutorials listener'


cd bagfiles
rosrun turtlesim turtlesim_node
rosbaplay 2017-09-28-11-02-56.bag

cd bagfiles
rosrun turtlesim turtlesim_node
rosbaplay subset.bag


rosrun beginner_tutorials talker
rosrun beginner_tutorials reverser
rostopic echo /rchatter 


