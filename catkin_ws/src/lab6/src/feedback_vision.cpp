#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/String.h>
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>

//Global variables
float robot_x=0,robot_y=0,target_angle,robot_angle = 0;
//float x,y,theta;

/*
void posCallback(const std_msgs::String::ConstPtr& msg){
	string temp=msg->data;
	vector <string> tokens;
	stringstream check1(temp);
	string intermediate;
	while(getline(check1,intermediate,' ')){
		tokens.push_back(intermediate);
	}
	x=tokens[0];
	y=tokens[1];
	theta=tokens[2];
}
*/

void setTransform(float x,float y,float z,float qx,float qy,float qz,float qw){	
	//Creates a transform from the vision target to the "goal" frame, to be used to track the position of the target as the robot moves
	tf2_ros::TransformBroadcaster tfbr;
	geometry_msgs::TransformStamped mTt;
	mTt.header.stamp=ros::Time::now();
	mTt.header.frame_id="4x4_1";
	mTt.child_frame_id="target";
	mTt.transform.translation.x=x;
	mTt.transform.translation.y=y;
	mTt.transform.translation.z=z;
	mTt.transform.rotation.x=qx;
	mTt.transform.rotation.y=qy;
	mTt.transform.rotation.z=qz;
	mTt.transform.rotation.w=qw;
	tfbr.sendTransform(mTt);
}

int main(int argc, char** argv){

	//INITIALIZEATIONS
	//CLI Arguments
	float x = atof(argv[1]),y = atof(argv[2]),theta = (atof(argv[3])*3.1415)/180;

	//ROS Nodes & msgs
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;
	//ros::Subscriber subpos = node.subscribe("/pos",10,posCallback);
	ros::Publisher pubvel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	geometry_msgs::Twist cmd_vel;

	//TF2
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	tf2_ros::TransformBroadcaster tfBroadcaster;
	ros::Duration(3).sleep();
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::TransformStamped wTt;
	geometry_msgs::TransformStamped temp;
	
	//Creates a transform from the world to the desired target position under the name "goal"
	wTt.header.stamp = ros::Time::now();
	wTt.header.frame_id = "world";
	wTt.child_frame_id = "goal";
	wTt.transform.translation.x = x;
	wTt.transform.translation.y = y;
	wTt.transform.translation.z = 0;
	tf2::Quaternion q;
	q.setRPY(0, 0, theta);
	target_angle = theta;
	wTt.transform.rotation.x = q.x();
	wTt.transform.rotation.y = q.y();
	wTt.transform.rotation.z = q.z();
	wTt.transform.rotation.w = q.w();
	tfBroadcaster.sendTransform(wTt);
	
	temp=tfBuffer.lookupTransform("4x4_1","goal",ros::Time(0),ros::Duration(.5));
	float mx=temp.transform.translation.x,my=temp.transform.translation.y,mz=temp.transform.translation.z,mqx=temp.transform.rotation.x,mqy=temp.transform.rotation.y,mqz=temp.transform.rotation.z,mqw=temp.transform.rotation.w;
	setTransform(mx,my,mz,mqx,mqy,mqz,mqw);
	
	//Robot movement parameters
	double speed=1,maxspeed=.075,radius=0.05,roll,pitch,yaw;
	ros::Rate rate(10.0);

	//DRIVE LOOPS
	//Position
	while (node.ok()){
		setTransform(mx,my,mz,mqx,mqy,mqz,mqw);
		ros::spinOnce();

		//Gets transform of the target with respect to the world
		try{
			transformStamped = tfBuffer.lookupTransform("world", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
        	
		
		//Set cmd_vel
		cmd_vel.angular.z = (0.5 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x));
                cmd_vel.linear.x = 0.32325 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

		//Publish
		pubvel.publish(cmd_vel);
		ROS_INFO("Dist: %f, Lx: %f",sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2)),cmd_vel.linear.x);

		//Exit condition
		if(cmd_vel.linear.x <= 0.1) {
			break;
		}
		rate.sleep();
	}

	//Set linear velocity to 0 for angle correction
	cmd_vel.linear.x=0;

	//Angle Correction
	while(node.ok()){
		setTransform(mx,my,mz,mqx,mqy,mqz,mqw);
		ros::spinOnce();

		//Get transforms
		try{
			transformStamped = tfBuffer.lookupTransform("world", "target",ros::Time(0));
			tf::Quaternion quat(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);
			tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		


		//Set cmv_vel
		speed=0.0625*yaw;
		if(speed>=maxspeed){
			speed=maxspeed;
		}
		cmd_vel.angular.z=speed;

		//Publish
		pubvel.publish(cmd_vel);
		ROS_INFO("Angle: %f,  Az: %f",transformStamped.transform.rotation.z,cmd_vel.angular.z);

		//Exit condition
		if(cmd_vel.angular.z <= 0.075){
			break;
		}

		rate.sleep();
	}

	ROS_INFO("I did it");
	return 0;
};
