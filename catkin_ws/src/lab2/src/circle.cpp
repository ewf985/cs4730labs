#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv){
	//Initialize ROS
	ros::init(argc,argv,"circle_node");
	ros::NodeHandle pubH;
	//Setup publisher to send commands on topic "/turtle1/cmd_vel"
	ros::Publisher pub = pubH.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);\
	geometry_msgs::Twist msg;
	//Rate to send commands at 1 second
	ros::Rate r(1);

	//Wait for a subscriber to connect before continuing 
	while (0 == pub.getNumSubscribers() && ros::ok()) {
        	ROS_INFO("Waiting for subscribers to connect");
        	r.sleep();
      	}

	double speed=2,radius=2;
	
	//Run until ros is shutdown
	while(ros::ok()){
		//Set the x (forward) speed and turning speed
		msg.linear.x=speed;
		msg.angular.z=speed/radius;
		//publish command to topic
		pub.publish(msg);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
