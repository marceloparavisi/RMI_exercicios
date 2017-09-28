#include "ros/ros.h"
#include "std_msgs/String.h"




ros::Publisher reverser_pub;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string text = msg->data.c_str();
  std::string reverseText;
  for(int i = text.length()-1; i >= 0; i--)
  {
	reverseText+= text.at(i);
  }

  std_msgs::String reverseMsg;
  reverseMsg.data = reverseText.c_str();
  reverser_pub.publish(reverseMsg);
  //ROS_INFO("I heard: [%s] reverse: %s", msg->data.c_str(), reverseText.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "reverser");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  reverser_pub = n.advertise<std_msgs::String>("rchatter", 1000);

  ros::spin();

  return 0;
}
