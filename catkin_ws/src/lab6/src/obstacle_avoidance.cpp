#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

//Global variables
bool leftBumper,rightBumper;

//Callback function for when messages are recieved 
void callbackl(const std_msgs::Bool::ConstPtr& msg){
	//Read left bumper from message
	leftBumper = msg->data;
	ROS_INFO("Left Bumper:\t%d",leftBumper);
}	

void callbackr(const std_msgs::Bool::ConstPtr& msg){
	//Read right bumper from message
	rightBumper = msg->data;
	ROS_INFO("Right Bumper:\t%d",rightBumper);
}	

int main(int argc,char **argv){
	//Setup ROS
	ros::init(argc,argv,"obstacle_avoidance_node");
	ros::NodeHandle nh;

	//Subscribers, publishers, and msg
	ros::Subscriber subl = nh.subscribe("bumperl",10,&callbackl);
	ros::Subscriber subr = nh.subscribe("bumperr",10,&callbackr);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("obstacle_avoidance/cmd_vel",10);
	geometry_msgs::Twist cmd_vel;

	//cmd_vel variables;
	float speed=.125;
	bool flag=false,prevFlag=false;

	//Rate for loop
	ros::Rate r(10);

	//Loop
	while(ros::ok()){
		if(leftBumper){
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=-speed;
			flag=true;
		}else if(rightBumper){
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=speed;
			flag=true;
		}
		else{
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=0;
			flag=false;
		}
		if(flag || prevFlag){
			ROS_INFO("\ncmd_vel.linear.x =\t%f\ncmd_vel.angular.z =\t%f",cmd_vel.linear.x,cmd_vel.angular.z);
			pub.publish(cmd_vel);
		}
		prevFlag=flag;
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
