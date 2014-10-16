/*
 * move_to_simple_server.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: ace
 */

#include "simple_service/move_to_simple_server.h"

namespace simple_service {

MoveToSimpleServer::MoveToSimpleServer(std::string cmd_vel_topi_name) :
_server(_nh, "move_to_simple", boost::bind(&MoveToSimpleServer::goalCallback, this, _1), false) ,
_tf_listener(ros::Duration(2))
{
	_cmd_vel_pub = _nh.advertise <geometry_msgs::Twist> (cmd_vel_topi_name, 1);

	ROS_INFO("Starting server...");
	_server.start();
	ROS_INFO("Server is up and running.");
}

MoveToSimpleServer::~MoveToSimpleServer()
{

}

void MoveToSimpleServer::goalCallback(const MoveToSimpleGoalConstPtr& goal)
{
	ROS_INFO("Got a new goal to work on... Hmmm...");

	int driving_direction = 1;

	if(goal->driving_direction == MoveToSimpleGoal::REVERSE)
		driving_direction = -1;

	geometry_msgs::PoseStamped start_pose = getCurrentPose();

	geometry_msgs::PoseStamped current_pose;
	current_pose.header = start_pose.header;
	current_pose.pose = start_pose.pose;

	geometry_msgs::Point final_point;
	float angle;

	// Simple Transformation.
	// X = x.cos(theta) - y.sin(theta) + trans-x;
	// Y = x.sin(theta) + y.cos(theta) + trans-y;

	final_point.x = goal->goal_pose.pose.position.x * cos(2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)) -
					goal->goal_pose.pose.position.y * sin(2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)) +
					start_pose.pose.position.x;

	final_point.y = goal->goal_pose.pose.position.x * sin(2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)) +
					goal->goal_pose.pose.position.y * cos(2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)) +
					start_pose.pose.position.y;

	ros::Rate publish_rate(4);

	float xy_tolerance = goal->xy_tolerance > 0.075 ? goal->xy_tolerance : 0.075;

	while(!equals(final_point, current_pose.pose.position, xy_tolerance))
	{
		////////////////////////////
		///// Publish velocity /////
		////////////////////////////
		if(driving_direction == 1)
			angle = atan2(final_point.y - current_pose.pose.position.y, final_point.x - current_pose.pose.position.x);
		else
			angle = atan2(current_pose.pose.position.y - final_point.y, current_pose.pose.position.x - final_point.x);

		float error_fi = angle - (2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w));

		geometry_msgs::Twist resultant_velocity;

		if( sqrt( (final_point.x - current_pose.pose.position.x) * (final_point.x - current_pose.pose.position.x) +
				   (final_point.y - current_pose.pose.position.y) * (final_point.y - current_pose.pose.position.y)) < 0.75)
			resultant_velocity.linear.x = 0.15 * driving_direction;
		else
			resultant_velocity.linear.x = 0.3 * driving_direction;

		resultant_velocity.angular.z = (atan2( sin(error_fi), cos(error_fi)) * 0.66);

		if(resultant_velocity.angular.z > 1.5)
			resultant_velocity.angular.z = 1.5;
		else if(resultant_velocity.angular.z < -1.5)
			resultant_velocity.angular.z = -1.5;

		_cmd_vel_pub.publish(resultant_velocity);

		//ROS_INFO("Velocities: x, theta = (%f, %f)", resultant_velocity.linear.x, resultant_velocity.angular.z);
		//ROS_INFO("ANGLE: (%f) and [%f]", angle, (2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)));

		////////////////////////////////////
		///// Recalculate our position /////
		////////////////////////////////////

		current_pose = getCurrentPose();

		publish_rate.sleep();
	}

	// Execute the rotation behaviour.
	float goal_fi_relative = 2*atan2(goal->goal_pose.pose.orientation.z, goal->goal_pose.pose.orientation.w);
	float goal_fi = 2*atan2(start_pose.pose.orientation.z, start_pose.pose.orientation.w) + goal_fi_relative;
	goal_fi = atan2(sin(goal_fi), cos(goal_fi));

	float our_fi = 2*atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);

	float yaw_tolerance = goal->yaw_tolerance > 0.15 ? goal->yaw_tolerance : 0.15;;

	while(ros::ok())
	{
		current_pose = getCurrentPose();

		our_fi = 2*atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);

		float del_fi = atan2(sin(goal_fi - our_fi), cos((goal_fi - our_fi)));

		geometry_msgs::Twist resultant_velocity;

		resultant_velocity.angular.z = (del_fi * 0.66);

		if(resultant_velocity.angular.z > 1.5)
			resultant_velocity.angular.z = 1.5;
		else if(resultant_velocity.angular.z < -1.5)
			resultant_velocity.angular.z = -1.5;

		_cmd_vel_pub.publish(resultant_velocity);

		//ROS_INFO("ANGLE: goal_relative: %f", goal_fi_relative);
		//ROS_INFO("del_fi: (%f)", del_fi);

		publish_rate.sleep();

		if(fabs(del_fi) < yaw_tolerance )
			break;
	}

	ROS_INFO("Action succeeded!");
	_server.setSucceeded();

}

geometry_msgs::PoseStamped MoveToSimpleServer::getCurrentPose()
{
	ros::Time _now_stamp_ = ros::Time::now();
	
	tf::StampedTransform start_pose_in_tf;
	
	_tf_listener.waitForTransform("odom", "base_link", _now_stamp_, ros::Duration(2.0));
	try
	{
		_tf_listener.lookupTransform("odom", "base_link", _now_stamp_, start_pose_in_tf);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("TRANSFORMS ARE COCKED-UP PAL! Why is that :=> %s", ex.what());
	}
	
	tf::Vector3 start_position = start_pose_in_tf.getOrigin();
	tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

	geometry_msgs::PoseStamped start_pose;
	start_pose.header.stamp = start_pose_in_tf.stamp_;
	start_pose.header.frame_id = start_pose_in_tf.frame_id_;

	tf::pointTFToMsg(start_position, start_pose.pose.position);
	tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

	return start_pose;
}

bool MoveToSimpleServer::equals(geometry_msgs::Point a, geometry_msgs::Point b, float tolerance)
{
	if(fabs(a.x - b.x) < tolerance &&
		fabs(a.y - b.y) < tolerance)
	return true;

	else return false;

}

geometry_msgs::Point operator + (geometry_msgs::Point a, geometry_msgs::Point b)
{
	geometry_msgs::Point result;

	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;

	return result;
}

} // NAMESPACE simple_service //

int main(int argn, char* args[])
{
	ros::init(argn, args, "move_to_simple");

	simple_service::MoveToSimpleServer move_to_server;

	ros::spin();
}
