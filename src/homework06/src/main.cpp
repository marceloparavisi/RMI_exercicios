#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <tf/tf.h>
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Float64.h>
#include <mutex>




std::vector<geometry_msgs::Pose2D> pointsVector;
geometry_msgs::Pose2D odom;
sensor_msgs::LaserScan laser;
bool desviando = false;
int lim = 100;

time_t timer;
time_t timer2;
struct tm y2k = {0};
double seconds;

float resolution = 0.1;
int width = 100;
int height = 100;
int data[10000];

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


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser.header.seq = msg->header.seq;
    laser.header.stamp = msg->header.stamp;
    laser.header.frame_id= msg->header.frame_id;
    laser.angle_min = msg->angle_min;
    laser.angle_max = msg->angle_max;
    laser.angle_increment = msg->angle_increment;
    laser.time_increment = msg->time_increment;
    laser.scan_time = msg->scan_time;
    laser.range_min = msg->range_min;
    laser.range_max = msg->range_max;
    laser.ranges = msg->ranges;
    laser.intensities = msg->intensities;
    //ROS_INFO(" testing callback: (%f, %f, %f)", laser.angle_min, laser.angle_max, laser.angle_increment);
    
//    ROS_INFO("teste: %d", *std::min_element(laser.ranges.begin(), laser.ranges.end()));
//    ROS_INFO("teste: %f", high);


	if (msg->ranges.size() > 0)
	{
		/*for (int x = 0; x < width; x++)
			for (int y = 0; y < height; y++)
			{
				data[x*width+y]=0;
			}//*/
		int menorX = 10000;
		int maiorX = -10000;
		int menorY = 10000;
		int maiorY = -10000;
		float menorAng = 10000;
		float maiorAng = -1000;


		int fromX = int(odom.x/resolution); 
		int fromY = int(odom.y/resolution);
		for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
		{
			tf::Vector3 v(1,0,0);
			tf::Quaternion q;
			q.setRotation(tf::Vector3(0,0,1), odom.theta);
			v = tf::quatRotate(q,v);
			tf::Vector3 target(x-fromX, y-fromY,0);
			float dist = target.distance(tf::Vector3(0,0,0));
			float angle = v.dot(target)/dist;
			
//			if (dist < 5 )//&& angle > (odom.theta+msg->angle_min) && angle < (odom.theta+msg->angle_max))
			if (dist < 15 && angle > (msg->angle_max) && angle >0) //angle > (msg->angle_min) )
			{
				ROS_INFO("%f angle: %f %f %f",dist, (msg->angle_min), angle, (msg->angle_max));
				if (data[y*width+x] > 0)
					data[y*width+x]=data[y*width+x]-1;
			}
	//		else
	//			data[y*width+x]=0;
		}
		for(unsigned long i = 0; i < msg->ranges.size(); i++)
		if(msg->ranges[i]<10)		
		{


			
			//ROS_INFO(" i: %d/%lu dist: %f", i, msg->ranges.size(), msg->ranges[i]);
			tf::Vector3 v(msg->ranges[i],0,0);
			tf::Quaternion q;
			if (menorAng > (odom.theta+msg->angle_min+i*msg->angle_increment))
				menorAng = odom.theta+msg->angle_min+i*msg->angle_increment; 
			if (maiorAng < (odom.theta+msg->angle_min+i*msg->angle_increment))
				maiorAng = odom.theta+msg->angle_min+i*msg->angle_increment;
			q.setRotation(tf::Vector3(0,0,1), odom.theta+msg->angle_min+i*msg->angle_increment);
			tf::Vector3 target = tf::quatRotate(q, v)+tf::Vector3(odom.x,odom.y,0);
//			tf::Vector3 target = tf::quatRotate(q, v);

			int x = int(target.x()/resolution);
			int y = int(target.y()/resolution);
			if (x < menorX) menorX = x;
			if (x > maiorX) maiorX = x;
			if (y < menorY) menorY = y;
			if (y > maiorY) maiorY = y;

			//data[width*y+x]=data[width*y+x]-1;
			
			
			if (x >=0 && y>=0 && x <= width && y <=height)
			{
				//ROS_INFO("i: %d target: %d, %d", i, x, y);
				data[y*width+x]=data[y*width+x]+3;
				//ROS_INFO(" --");
			}
		}
		ROS_INFO(" X: %d - %d Y: %d - %d", menorX, maiorX, menorY, maiorY);
		ROS_INFO(" ANG: %f - %f", menorAng, maiorAng);
		
	}
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

	ros::init(argc, argv, "homework05");

	ros::NodeHandle n;


	// 2- ler /odom
	ros::Subscriber sub2 = n.subscribe("odom", 1, odometryCallback);

    	// inscreve no laser
	ros::Subscriber sub3 = n.subscribe("scan", 100, laserCallback);

	

	ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("hmmi", 1);
	// inicializa grid com zero
	for (int x = 0; x < width; x++)
	for (int y = 0; y < height; y++)
	{
		data[x*width+y]=0;
	}


        ros::Rate loop_rate(10);

	while(ros::ok())
	{


		ROS_INFO(" Vector: %d", pointsVector.size());
		
		nav_msgs::OccupancyGrid hmmi;
		//hmmi.header.seq =; // ????
		hmmi.header.stamp =ros::Time::now();
		hmmi.header.frame_id = "odom";
		hmmi.info.map_load_time=ros::Time::now();
		hmmi.info.resolution = resolution;
		hmmi.info.width = width;
		hmmi.info.height = height;
		hmmi.info.origin.position.x = 0;
		hmmi.info.origin.position.y = 0;
		hmmi.info.origin.position.z = 0;
		hmmi.info.origin.orientation.x = 0;
		hmmi.info.origin.orientation.y = 0;
		hmmi.info.origin.orientation.z = 0;
		hmmi.info.origin.orientation.w = 1;
		hmmi.data.resize(width * height);
		for (int i = 0; i < width*height; i++)
		{
			hmmi.data[i]=data[i];
		}
		ROS_INFO("publishing hmmi");
		map_pub.publish(hmmi);
		ROS_INFO("done");

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
