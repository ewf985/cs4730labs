#include "math.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#define PI 3.141592654

double currentPosX;
double currentPosY;
double currentPosW;
double currentPosZ;

//Odom Callback function
void callback(const nav_msgs::Odometry::ConstPtr& msg){
	currentPosX=msg->pose.pose.position.x;	//Position X
	currentPosY=msg->pose.pose.position.y;	//Position Y
	currentPosW=msg->pose.pose.orientation.w;	//Orientation X comp
	currentPosZ=msg->pose.pose.orientation.z;	//Orientation Y comp	
	//ROS_INFO("Pos x,y,z: %f, %f, %f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	//ROS_INFO("ori x,y,z,w: %f,%f,%f,%f",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
}

//determine the final coordinate point
void findTargetPos(double* targetPosX,double* targetPosY,const double distance){
	*targetPosX=currentPosX;
	*targetPosY=currentPosY;
	*targetPosX+=distance*currentPosW;
	*targetPosY+=distance*currentPosZ;
	return;
}

//determine the angle to turn to
void findTargetAngle(double* targetAngleW,double* targetAngleZ,const double angle){
	double target;
	target=angle+(atan(currentPosZ/currentPosW)*(180/PI));
	*targetAngleW=cos(target);
	*targetAngleZ=sin(target);
	return;
}

//check to see if the distnace has been traveled to
bool checkDistance(const double targetX,const double targetY,const double err){
	ROS_INFO("Current Pos: %f,%f\nTarget Range: %f-%f,%f-%f",currentPosX,currentPosY,targetX-err,targetX+err,targetY-err,targetY+err);
	if(currentPosX>=(targetX-err) && currentPosX<=(targetX+err) && currentPosY>=(targetY-err) && currentPosY<=(targetY+err)){
		//ROS_INFO("Return: True");
		return true;
	}
	//ROS_INFO("Return: False");
	return false;
}

//check to see if the angle has been reached
bool checkAngle(const double targetW,const double targetZ,const double err){
	double currentAngle=(180/PI)*atan(::currentPosZ/::currentPosW),targetAngle=(180/PI)*atan(targetZ/targetW);
	ROS_INFO("Current Angle: %f\nTarget Range: %f-%f",currentAngle,targetAngle-err,targetAngle+err);
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
	double targetX,targetY,targetW,targetZ,distance=1,angle=90,speed=.1,radius=.005;
	
	while(ros::ok()){
		//set speeds to 0
		msg.linear.x=0;
		msg.angular.z=0;
		pub.publish(msg);
		ros::spinOnce();
		r.sleep();
		//get and go to the next position
		findTargetPos(&targetX,&targetY,distance);
		while(!checkDistance(targetX,targetY,distance*.001) && ros::ok()){
			//ROS_INFO("Driving");
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
		findTargetAngle(&targetW,&targetZ,angle);
		while(!checkAngle(targetW,targetZ,1) && ros::ok()){
			//ROS_INFO("Turning");
			msg.linear.x=speed/100;
			msg.angular.z=-(speed/100)/radius;
			pub.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
	}
	return 0;
}