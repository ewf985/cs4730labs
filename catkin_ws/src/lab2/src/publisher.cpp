#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc,char **argv){
	//Setup ROS
	ros::init(argc,argv,"publisher_node");
	ros::NodeHandle nh;
	//setup publisher to send messages to topic "counter"
	ros::Publisher bub = nh.advertise<std_msgs::String>("counter",1000);
	//set rate to 1 second
	ros::Rate r(1);

	//Waits for subscribers before posting messages
	while (0 == bub.getNumSubscribers() && ros::ok()) {
          ROS_INFO("Waiting for subscribers to connect");
          r.sleep();
      	}


	int i=0;

	//Posts messages
	while(ros::ok()){
		std_msgs::String str;
		std::stringstream ss;
		ss<<"Message No. #"<<i;
		str.data=ss.str();
		ROS_INFO("%s",str.data.c_str());
		bub.publish(str);
		ros::spinOnce();
		r.sleep();
		++i;
	}
	return 0;
}
