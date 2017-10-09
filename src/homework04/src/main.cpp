#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/tf.h>




std::vector<geometry_msgs::Pose2D> pointsVector;
geometry_msgs::Pose2D odom;


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
	//ROS_INFO("GOT ODOM: position (%f, %f, %f) orient: %f, %f, %f)", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z,  msg->pose.pose.orientation.x,msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
	odom.x = msg->pose.pose.position.x;
	odom.y = msg->pose.pose.position.y;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 mat(q);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	odom.theta = yaw;
	//ROS_INFO("yaw: %f",yaw);
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
	ros::Subscriber sub2 = n.subscribe("odom", 1, odometryCallback);

	ros::Publisher commandsTwist = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/safety_controller", 1);

        ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ROS_INFO(" Vector: %d", pointsVector.size());
		if (pointsVector.size() > 0)
		{
			double rot1 = atan2(pointsVector[0].y-odom.y, pointsVector[0].x-odom.x)-odom.theta;
			double dist = sqrt ((pointsVector[0].x-odom.x)*(pointsVector[0].x-odom.x) + (pointsVector[0].y-odom.y)*(pointsVector[0].y-odom.y));
			int multAngle = rot1 / 0.06;
ROS_INFO(" p(%f, %f) odom(%f,%f)",pointsVector[0].x, pointsVector[0].y,odom.x,odom.y);
ROS_INFO("ROT1: %f odom.theta: %f m: %d dist: %f", rot1, odom.theta,  multAngle, dist);
			if ((multAngle == 0) || (dist < 0.1))
			{
				if (dist > 0.2)
					dist = 0.2;
				else if (dist > 0.1)
					dist = 0.1;
				else
				{
					dist = 0;
					double rot2 = pointsVector[0].theta - odom.theta;
					multAngle = rot2 / 0.06;
ROS_INFO("rot2: %f m2: %d ", rot2, multAngle);
					if (multAngle == 0)
						pointsVector.erase(pointsVector.begin());
				}
			}
			else
				dist = 0;
			if (multAngle > 5)
				multAngle = 5;
			
			geometry_msgs::Twist cmd;
			cmd.linear.x = dist; 
			cmd.linear.y = 0; 
			cmd.linear.z = 0; 
			cmd.angular.x = 0; 
			cmd.angular.y = 0; 
			cmd.angular.z = 0.1*multAngle; 

			commandsTwist.publish(cmd);
						
			// 3- aplicar rot1
			// 4- aplicar trans1
			// 5- aplicar rot2
			// 6- se chegou no destino, remove da fila de pose
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
