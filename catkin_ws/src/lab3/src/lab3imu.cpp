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

//Create variables
float turtle2_angle,turtle2_x,turtle2_y,turtle1_angle = 0;

//Callback function for the target position. The target position transform gets created here
void pose1Callback(float x, float y, float theta){				
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	//Store the time, frame ids, and the translation and rotation data into this transform
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "target";
	transformStamped.transform.translation.x = x;
	transformStamped.transform.translation.y = y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, theta);
	turtle1_angle = theta;
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	//Broadcast the transform
	br.sendTransform(transformStamped);
}

//Callback function for the robots position. 
void pose2Callback(const nav_msgs::Odometry::ConstPtr& msg){
	//Reads x and y from odometry and stores into global variable
	turtle2_x=msg->pose.pose.position.x;
	turtle2_y=msg->pose.pose.position.y;
}
//Callback function for the robots orientation from the IMU
void pose3Callback(const std_msgs::String::ConstPtr& msg){
	//Reads yaw from imu and stores into global variable
	turtle2_angle=atof(msg->data.c_str());
	turtle2_angle=-turtle2_angle;
}
//Create the transform for the current position and orientation of the robot
void setTransform(void){
	//Creates a transfrom from the x, y, and yaw collected in the subscriber callback functions
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "turtle1";
	
	//Set transform x,y,z
	transformStamped.transform.translation.x = turtle2_x;
	transformStamped.transform.translation.y = turtle2_y;
	transformStamped.transform.translation.z = 0.0;
	
	//Convert yaw to quaternion
 	tf::Quaternion q(0,0,0,0);
	q.setRPY(0,0,turtle2_angle);	

	//Set transform quaternion
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
	//Get command line input from running the program of where the target position is
	float x = atof(argv[1]),y = atof(argv[2]),theta = (atof(argv[3])*3.1415)/180;
	
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;
	//Subscribe to the odom and imu topics
	ros::Subscriber sub2 = node.subscribe("/odom", 10, &pose2Callback);
	ros::Subscriber sub3 = node.subscribe("/imu", 10, &pose3Callback);
	//Setup the publisher for cmd_vel
	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Twist vel_msg;
	double speed,radius=0.05;

	ros::Rate rate(10.0);
	//This loop is to get the robot to the specified point
	while (node.ok()){
		//Spin to read from the subscriptions
		ros::spinOnce();
		//Run this function to create the transform for the current robot position
		setTransform();
		//Run this to create the transform for the target position based on the input
		pose1Callback(x,y,theta);
		//Try to get the transform between the two points: target and current robot position
		try{
			transformStamped = tfBuffer.lookupTransform("turtle1", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		//Calculate and set the angular and linear velocity based on the transform found
		vel_msg.angular.z = (0.5 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x));
		vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
		turtle_vel.publish(vel_msg);

		ROS_INFO("Az: %f,  Lx: %f",vel_msg.angular.z,vel_msg.linear.x);

		//If the calculated velocities are very small, the robot is considered to be at the location and should stop moving
		if(vel_msg.linear.x <= 0.1) {
			break;
		}

		rate.sleep();
	}
	//This loop is to get the robot to the correct orientation
	while(node.ok()){
		//Spin to read from the subscriptions
		ros::spinOnce();
		//Run this function to create the transform for the current robot position
		setTransform();
		//Run this to create the transform for the target position based on the input
		pose1Callback(x,y,theta);
		//Try to get the transform between the two points: target and current robot position
		try{
			transformStamped = tfBuffer.lookupTransform("turtle1", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		//calcualte and set the angular and linear speed
		speed=0.25*pow((turtle2_angle-turtle1_angle),2);
		if(speed>.1){
			speed=.1;
		}
		vel_msg.linear.x=speed;
		vel_msg.angular.z=speed/radius;
		turtle_vel.publish(vel_msg);

		ROS_INFO("Az: %f,  Lx: %f",vel_msg.angular.z,vel_msg.linear.x);
		
		//If the calculated velocities are very small, the robot is considered to be at the location and should stop moving
		if(vel_msg.angular.z <= 0.075){
			break;
		}
		rate.sleep();
	}
	//Announce that the robot thinks it is at the specified point
	ROS_INFO("I did it");
	return 0;
};
