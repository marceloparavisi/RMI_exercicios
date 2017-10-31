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
int deslX = 5;
int deslY = 5;
int data[10000];
std::mutex mutex;

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
		for(unsigned long i = 0; i < msg->ranges.size(); i++){

			ROS_INFO(" i: %d/%lu dist: %f", i, msg->ranges.size(), msg->ranges[i]);
			tf::Vector3 v(1,0,0);
			tf::Quaternion q;
			q.setRotation(tf::Vector3(0,0,1), odom.theta+msg->angle_min+i*msg->angle_increment);
			tf::Vector3 target = tf::quatRotate(q, v)+tf::Vector3(odom.x,odom.y,0);
			ROS_INFO("i: %d target: %f, %f", i, target.x(), target.y());
			data[int(target.x()*width+target.y())]=255;
		}
		
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

	// 1- ler topico e armazenar numa fila de pose
	ros::Subscriber sub = n.subscribe("setPoint", 10, setPoint);
	// 2- ler /odom
	ros::Subscriber sub2 = n.subscribe("odom", 1, odometryCallback);
    	// inscreve no laser
	ros::Subscriber sub3 = n.subscribe("scan", 100, laserCallback);

	ros::Publisher commandsTwist = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/safety_controller", 1);

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
           
            		float high = 100000;

			ROS_INFO(" Looking for min distance %d", laser.ranges.size());
			if (laser.ranges.size() > lim)
			{
				for(int i = lim; i < laser.ranges.size()-lim; i++){
				 	high = std::min(high, float(laser.ranges[i]));
				}
			}
            		ROS_INFO(" min: %f", high);
			if (high < 1.5)
			{
				desviando = true;
				geometry_msgs::Twist cmd;

				time(&timer);
				timer2 = timer;

				while(difftime(timer, timer2) < 0.8){
					//rotaciona
					cmd.linear.x = 0; 
					cmd.linear.y = 0; 
					cmd.linear.z = 0; 
					cmd.angular.x = 0; 
					cmd.angular.y = 0; 
					cmd.angular.z = 0.2; 

					commandsTwist.publish(cmd);
					time(&timer);
				}

			}
			if (high >= 1.5 && desviando){
				time(&timer);
				timer2 = timer;
				while(difftime(timer, timer2) < 1){
					//rotaciona
					cmd.linear.x = 0.2; 
					cmd.linear.y = 0; 
					cmd.linear.z = 0; 
					cmd.angular.x = 0; 
					cmd.angular.y = 0; 
					cmd.angular.z = 0; 

					commandsTwist.publish(cmd);
					time(&timer);
				}
				desviando = false;  
			}
			ROS_INFO("min_distance: %f, %d", high, desviando);
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
						
			// 3- aplicar rot1
			// 4- aplicar trans1
			// 5- aplicar rot2
			// 6- se chegou no destino, remove da fila de pose
			// pegar info do laser
			// verificar se na regiao central a distancia e menor que um valor estabelecido
			// se sim, coloca um ponto correto na fila
			// anda mais um pouco pra frente

		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
