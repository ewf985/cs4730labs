#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>


void pose1Callback(const turtlesim::PoseConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "turtle1";
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

void pose2Callback(const turtlesim::PoseConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "turtle2";
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf2_broadcaster");
	ros::NodeHandle node;

	//Spawn in a new turtle
	ros::service::waitForService("spawn");
	ros::ServiceClient spawner =
	node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn turtle;
	turtle.request.x = 4;
	turtle.request.y = 2;
	turtle.request.theta = 0;
	turtle.request.name = "turtle2";
	spawner.call(turtle);


	ros::Subscriber sub1 = node.subscribe("turtle1/pose", 10, &pose1Callback);
	ros::Subscriber sub2 = node.subscribe("turtle2/pose", 10, &pose2Callback);

	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(10.0);
	while (node.ok()){
		ros::spinOnce();
		geometry_msgs::TransformStamped transformStamped;
		try{
			transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::Twist vel_msg;

		vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,transformStamped.transform.translation.x);
		vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
		turtle_vel.publish(vel_msg);

		
		rate.sleep();
	}
	return 0;
};
