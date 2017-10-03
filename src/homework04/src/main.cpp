#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry>
#include <math.h>



std::vector<geometry_msgs::Pose2D> pointsVector;



void setPoint(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	geometry_msgs::Pose2D p;
	p.x = msg->x;
	p.y = msg->y;
	p.theta = msg->theta;
	pointsVector.push_back(p);

	ROS_INFO(" new point addeed: (%f, %f, %f )", p.x, p.y, p.theta);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("GOT ODOM: position (%f, %f, %f) orient: %f, %f, %f)", msg->pose->pose->position.x,msg->pose->pose->position.y, msg->pose->pose->position.z,  msg->pose->pose->orientation.x,msg->pose->pose->orientation.y, msg->pose->pose->orientation.z, msg->pose->pose->orientation.w);  

}




void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double linearX = msg->linear.x;
  double linearY = msg->linear.y;
  double linearZ = msg->linear.z;

  double angularX = msg->angular.x; // drop
  double angularY = msg->angular.y; // drop
  double angularZ = msg->angular.z;

  //ROS_INFO("Linear Speed: [%f, %f, %f] angular: [%f, %f, %f] ", linearX, linearY, linearZ, angularX, angularY, angularZ);


  double L = 1;
  double R = 0.1;

  double w = angularZ;

  double v  = std::sqrt(linearX*linearX+linearY*linearY);
  double vl = (2*v-w*L)/(2*R);
  double vr = (2*v+w*L)/(2*R);
  ROS_INFO(" vl: %f   vr: %f", vl, vr);
}



/*
void poseCallback(const homework03::Input::ConstPtr& msg)
{
  double x = msg->x;
  double y = msg->y;
  double theta = msg->theta;
  double ticksL = msg->ticksL;
  double ticksR = msg->ticksR;


  double L = 1;
  double R = 0.1;
  int N = 16;

  double Dc = M_PI*R*(ticksL+ticksR)/N;
  double Dr = 2*M_PI*R*ticksR/N;
  double Dl = 2*M_PI*R*ticksL/N;

  double x2  = x+Dc*cos(theta);
  double y2 = y+Dc*sin(theta);
  double theta2 =theta + (Dr-Dl)/L;
  ROS_INFO(" x2: %f  y2: %f, theta2: %f", x2, y2, theta2);
}
*/


int main(int argc, char **argv)
{



	ros::init(argc, argv, "homework04");

	ros::NodeHandle n;

	// 1- ler topico e armazenar numa fila de pose
	ros::Subscriber sub = n.subscribe("setPoint", 10, setPoint);
	// 2- ler /odom
	ros::Subscriber sub2 = n.subscribe("input", 1, odometryCallback);

	ros::spin();

	while(ros::ok())
	{

		
		// 3- aplicar rot1
		// 4- aplicar trans1
		// 5- aplicar rot2
		// 6- se chegou no destino, remove da fila de pose
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
