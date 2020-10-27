#include "math.h"
#include <tf/tf.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#define PI 3.141592654

double currentPosX;
double currentPosY;
double currentPosYaw;

//Odom Callback function
void callback(const nav_msgs::Odometry::ConstPtr& msg){
	currentPosX=msg->pose.pose.position.x;	//Position X
	currentPosY=msg->pose.pose.position.y;	//Position Y
	//ROS_INFO("Pos x,y,z: %f, %f, %f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	
	//Convert to quaternion
 	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentPosYaw = yaw;
}

int main(int argc,char **argv){
	//setup ROS
	ros::init(argc,argv,"polygon_node");
	ros::NodeHandle pubH,subH;
	//setup publisher to send on topics
	ros::Publisher pub = pubH.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	ros::Subscriber sub = subH.subscribe("/odom",1000,callback);
	geometry_msgs::Twist msg;
	//run 100 times/second
	ros::Rate r(100);
	
	//Wait for subscribers to conenct before continuing
	while (0 == pub.getNumSubscribers() && ros::ok()) {
        	ROS_INFO("Waiting for subscribers to connect");
        	r.sleep();
      	}
	return 0;
}

