#include "ros/ros.h"
#include "std_msgs/String.h"

void counterCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("%s",msg->data.c_str());
}	

int main(int argc,char **argv){
	ros::init(argc,argv,"subscriber_node");
	ros::NodeHandle nh;

	ros::Subscriber sbub = nh.subscribe("counter",1000,counterCallback);

	ros::spin();
	return 0;
}
