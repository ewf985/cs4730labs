#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

//Create variables
float turtle2_angle,turtle1_angle = 0;

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
	//Broadcast this transform
	br.sendTransform(transformStamped);
}
//Callback function for the robots position. The robots current position transform gets created here
void pose2Callback(const nav_msgs::Odometry::ConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	//Store the time, frame ids, and the translation and rotation data into this transform
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "turtle1";
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = 0.0;
	
	//Convert to quaternion
 	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	turtle2_angle = yaw;
	
	//Display the current position and orientation 
	ROS_INFO("x,y,t: %f,%f,%f",msg->pose.pose.position.x,msg->pose.pose.position.y,yaw);

	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
	//Broadcast this transform
	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
	float x = atof(argv[1]),y = atof(argv[2]),theta = (atof(argv[3])*3.1415)/180;
	
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;
	//Subscribe to the odometry and send the data to the robot callback function
	ros::Subscriber sub2 = node.subscribe("/odom", 10, &pose2Callback);   
	//Set up publisher to talk on cmd_vel
	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(10.0);
	while (node.ok()){
		//Listen to the subscribed topics
		ros::spinOnce();
		//Set up the target position transform using the specified coordinate and orientation
		pose1Callback(x,y,theta);
		geometry_msgs::TransformStamped transformStamped;
		//Try to get the transform between the two points: target and current robot position
		try{
			transformStamped = tfBuffer.lookupTransform("turtle1", "target",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::Twist vel_msg;
		//Calculate and set the angular and linear velocity based on the transform found
		vel_msg.angular.z = (4.0 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x)) + (turtle2_angle-turtle1_angle);
		vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
		turtle_vel.publish(vel_msg);

		////ROS_INFO("Az: %f,  Lx: %f",vel_msg.angular.z,vel_msg.linear.x);

		//If the calculated velocities are very small, the robot is considered to be at the location and should stop moving
		if(vel_msg.angular.z <= 0.01 && vel_msg.linear.x <= 0.01) {
			break;
		}

		
		rate.sleep();
	}
	//Announce that the robot thinks it is at the specified point
	ROS_INFO("I did it");
	return 0;
};
