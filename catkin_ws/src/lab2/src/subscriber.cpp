#include "ros/ros.h"
#include "std_msgs/String.h"

//Callback function for when messages are recieved 
void counterCallback(const std_msgs::String::ConstPtr& msg){
	//Print to screen
	ROS_INFO("%s",msg->data.c_str());
}	

int main(int argc,char **argv){
	//Setup ROS
	ros::init(argc,argv,"subscriber_node");
	ros::NodeHandle nh;
	//Subscribe to topic "counter"
	ros::Subscriber sbub = nh.subscribe("counter",1000,counterCallback);
	//run ros forever
	ros::spin();
	return 0;
}
