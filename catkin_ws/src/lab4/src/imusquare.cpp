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

float turtle2_angle,turtle2_x,turtle2_y,turtle1_angle = 0;

void pose1Callback(float x, float y, float theta){				
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

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

	br.sendTransform(transformStamped);
}

void pose2Callback(const nav_msgs::Odometry::ConstPtr& msg){
	//Reads x and y from odometry
	turtle2_x=msg->pose.pose.position.x;
	turtle2_y=msg->pose.pose.position.y;
}

void pose3Callback(const std_msgs::String::ConstPtr& msg){
	//Reads yaw from imu
	turtle2_angle=atof(msg->data.c_str());
	turtle2_angle=-turtle2_angle;
}

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
	
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;

	ros::Subscriber sub2 = node.subscribe("/odom", 10, &pose2Callback);
	ros::Subscriber sub3 = node.subscribe("/imu", 10, &pose3Callback);

	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  //  /turtle1/cmd_vel

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Twist vel_msg;
	double speed,radius=0.05;

	ros::Rate rate(10.0);

	float x=0,y=0,theta=0,len=.75;

while(node.ok()){

	if(x==0 && y==0){
		x=len;
		y=0;
		theta=0;
	}else if(x==len && y==0){
		y=len;
	}else if(x==len && y==len){
		x=0;
	}else if(x==0 && y==len){
		y=0;
	}

	while (node.ok()){
		ros::spinOnce();
		setTransform();
		pose1Callback(x,y,theta);
		try{
			transformStamped = tfBuffer.lookupTransform("turtle1", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		vel_msg.angular.z = (0.5 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x));
		vel_msg.linear.x = 0.32325 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
		turtle_vel.publish(vel_msg);

		ROS_INFO("Az: %f,  Lx: %f",vel_msg.angular.z,vel_msg.linear.x);

		if(vel_msg.linear.x <= 0.1) {
			break;
		}

		rate.sleep();
	}
	
	if(theta==0){
		theta=90;
	}else if(theta==90){
		theta=180;
	}else if(theta==180){
		theta=-90;
	}else if(theta==-90){
		theta=0;
	}

	while(node.ok()){
		ros::spinOnce();
		setTransform();
		pose1Callback(x,y,theta);
		try{
			transformStamped = tfBuffer.lookupTransform("turtle1", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		speed=0.125*(turtle2_angle-theta);
		if(speed>.1){
			speed=.1;
		}
		vel_msg.linear.x=speed;
		vel_msg.angular.z=speed/radius;
		turtle_vel.publish(vel_msg);
		ROS_INFO("Az: %f,  Lx: %f",vel_msg.angular.z,vel_msg.linear.x);
		if(vel_msg.angular.z <= 0.075){
			break;
		}
		rate.sleep();
	}
	ROS_INFO("I did it");

	
	}
	return 0;
};
