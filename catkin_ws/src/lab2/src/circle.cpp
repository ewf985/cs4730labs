#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv){

	ros::init(argc,argv,"circle_node");
	ros::NodeHandle pubH;
	ros::Publisher pub = pubH.advertise<geometry_msgs::Twist>("/cmd_vel",1000);\
	geometry_msgs::Twist msg;
	ros::Rate r(1);

	while (0 == pub.getNumSubscribers() && ros::ok()) {
        	ROS_INFO("Waiting for subscribers to connect");
        	r.sleep();
      	}

	double speed=.1,radius=.005;
	
	while(ros::ok()){
		msg.linear.x=speed;
		msg.angular.z=speed/radius;
		pub.publish(msg);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
