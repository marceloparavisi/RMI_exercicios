#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "homework03/Input.h"



ros::Publisher reverser_pub;

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


int main(int argc, char **argv)
{

  ros::init(argc, argv, "wheelSpeed");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, commandCallback);
  ros::Subscriber sub2 = n.subscribe("input", 1000, poseCallback);

  ros::spin();

  return 0;
}
