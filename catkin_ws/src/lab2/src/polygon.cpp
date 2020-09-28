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

//check to see if the distnace has been traveled to
bool checkDistance(const double startX,const double startY,const double target){
	double distance=sqrt(((currentPosX-startX)^2)+((currentPosY-startY)^2));
	if(distance>=target){
		return true;
	}
	return false;
}

//check to see if the angle has been reached
bool checkAngle(const double target,const double err){
	double currentAngle=(180/PI)*currentPosYaw;
	double targetAngle=target;
	ROS_INFO("Current Angle: %f\nTarget Range: %f to %f",currentAngle,targetAngle-err,targetAngle+err);
	if(currentAngle>=(targetAngle-err) && currentAngle<=(targetAngle+err)){
		//ROS_INFO("Return: True");
		return true;
	}
	//ROS_INFO("Return: False");
	return false;
}

//================================================================
int main(int argc,char **argv){

	ros::init(argc,argv,"circle_node");
	ros::NodeHandle pubH,subH;
	ros::Publisher pub = pubH.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	ros::Subscriber sub = subH.subscribe("/odom",1000,callback);
	geometry_msgs::Twist msg;
	ros::Rate r(100);

	while (0 == pub.getNumSubscribers() && ros::ok()) {
        	ROS_INFO("Waiting for subscribers to connect");
        	r.sleep();
      	}

	int i;
	double startX,startY,targetYaw,distance=1,angle=72,yawErr=1,speed=.1,radius=.005;
	
	while(ros::ok()){
		//set speeds to 0
		msg.linear.x=0;
		msg.angular.z=0;
		pub.publish(msg);
		ros::spinOnce();
		r.sleep();
		//set start position and move a distance from it
		startX=currentPosX;
		startY=currentPosY;
		while(!checkDistance(startX,startY,distance) && ros::ok()){
			msg.linear.x=speed;
			pub.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
		//set speeds to 0
		msg.linear.x=0;
		pub.publish(msg);
		ros::spinOnce();
		r.sleep();
		//get and go to next angle
		targetYaw = (180/PI)*currentPosYaw + angle;
		if(targetYaw > 180) {
			targetYaw = targetYaw -360;
		}
		while(!checkAngle(targetYaw,yawErr) && ros::ok()){
			//ROS_INFO("Turning");
			msg.linear.x=speed/100;
			msg.angular.z=(speed/100)/radius;
			pub.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
	}
	return 0;
}

