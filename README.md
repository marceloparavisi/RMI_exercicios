

## To compile this project:

catkin_make

## Commands to run Homework 01:

	source devel/setup.bash
	roslaunch ros_robotics ddrobot_rviz.launch model:=dd_robot.urdf gui:=true

	urdf_to_graphiz src/ros_robotics/urdf/dd_robot.urdf 
	evince dd_robot.pdf

	roslaunch ros_robotics ddrobot_gazebo.launch 


## Commands to run Homework 02:

	source devel/setup.bash
	rosrun beginner_tutorials talker
	rosrun beginner_tutorials listener


	cd bagfiles
	rosrun turtlesim turtlesim_node
	rosbaplay 2017-09-28-11-02-56.bag

	cd bagfiles
	rosrun turtlesim turtlesim_node
	rosbaplay subset.bag


	rosrun beginner_tutorials talker
	rosrun beginner_tutorials reverser
	rostopic echo /rchatter 


## Commands to run Homework 03:

rosrun homework03 wheelSpeed
rostopic pub /cmd_vel geometry_msgs/Twist "{linear:  {x: 1.4,  y: 0.0,  z: 0.0}, angular:  {x: 0.0,  y: 0.0,  z: 0.0}}" 
rostopic pub /input homework03/Input "{x: 1.0, y: 0.0, theta: 0.0, ticksL: 16.0, ticksR: 16.0}"

## Commands to run Homework 04:

	roslaunch mybot_gazebo mybot_world.launch
	roslaunch mybot_description mybot_rviz.launch
	rosrun tf view_frames
	evince frames.pdf


	roslaunch tortoisebot tortoisebot.launch 
	rviz src/tortoisebot/rviz/basic.rviz
