#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"

//Global variables
float robot_x,robot_y,target_angle,robot_angle = 0;
bool leftBumper,rightBumper;

//Callback functions 
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

void poseCartCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Reads x and y from odometry
	robot_x=msg->pose.pose.position.x;
	robot_y=msg->pose.pose.position.y;
}

void poseYawCallback(const std_msgs::String::ConstPtr& msg){
	//Reads yaw from imu
	robot_angle=atof(msg->data.c_str());
	robot_angle=-robot_angle;
}

//Creates a transform from a provided x,y, and yaw
void setTransform(float x, float y, float yaw){				
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "target";
	transformStamped.transform.translation.x = x;
	transformStamped.transform.translation.y = y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	target_angle = yaw;
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

//Creates a transfrom from the x, y, and yaw collected in the subscriber callback functions
void getTransform(void){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "robot";
	
	//Set transform x,y,z
	transformStamped.transform.translation.x = robot_x;
	transformStamped.transform.translation.y = robot_y;
	transformStamped.transform.translation.z = 0.0;
	
	//Convert yaw to quaternion
 	tf::Quaternion q(0,0,0,0);
	q.setRPY(0,0,robot_angle);	

	//Set transform quaternion
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){

	//Initializations
	float x = atof(argv[1]),y = atof(argv[2]),theta = (atof(argv[3])*3.1415)/180;
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;
	ros::Subscriber subcart = node.subscribe("/odom", 10, &poseCartCallback);
	ros::Subscriber subyaw = node.subscribe("/imu", 10, &poseYawCallback);
	ros::Subscriber subl = node.subscribe("bumperl",10,&callbackl);
	ros::Subscriber subr = node.subscribe("bumperr",10,&callbackr);
	ros::Publisher pubvel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Twist cmd_vel;
	double speed,radius=0.05;
	ros::Rate rate(10.0);

	//Position Loop
	while (node.ok()){
		ros::spinOnce();

		//Get transforms
		getTransform();
		setTransform(x,y,theta);
		try{
			transformStamped = tfBuffer.lookupTransform("robot", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}


		//Set cmd_vel from transforms
		cmd_vel.angular.z = (0.5 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x));
		cmd_vel.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

		//Obstacle avoidance override
		if(leftBumper){
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=-speed/radius;
		}else if(rightBumper){
			cmd_vel.linear.x=speed;
			cmd_vel.angular.z=speed/radius;
		}

		//Publish
		pubvel.publish(cmd_vel);
		ROS_INFO("Az: %f,  Lx: %f",cmd_vel.angular.z,cmd_vel.linear.x);

		//Exit condition
		if(cmd_vel.linear.x <= 0.1) {
			break;
		}
		rate.sleep();
	}

	//Set linear velocity to 0 for angle correction
	cmd_vel.linear.x=0;

	//Angle Correction Loop
	while(node.ok()){
		ros::spinOnce();

		//Get transforms
		getTransform();
		setTransform(x,y,theta);
		try{
			transformStamped = tfBuffer.lookupTransform("robot", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}


		speed=0.125*(target_angle-robot_angle);
		if(speed>.1){
			speed=.1;
		}
		cmd_vel.angular.z=speed;

		//Publish
		pubvel.publish(cmd_vel);
		ROS_INFO("Az: %f,  Lx: %f",cmd_vel.angular.z,cmd_vel.linear.x);

		//Exit condition
		if(cmd_vel.angular.z <= 0.075){
			break;
		}

		rate.sleep();
	}

	ROS_INFO("I did it");
	return 0;
};
