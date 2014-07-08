/*
 * move_to_simple_server.h
 *
 *  Created on: Jun 23, 2014
 *      Author: ace
 */

#include <simple_service/MoveToSimpleAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <string>
#include <cmath>

#include <iostream>

#ifndef MOVE_TO_SIMPLE_SERVER_H_
#define MOVE_TO_SIMPLE_SERVER_H_

#define GOAL_TOLERANCE 0.1

class MoveToSimpleServer
{
	/**
	 * A NodeHandle for class' access.
	 */
	ros::NodeHandle _nh;

	/**
	 * The actual actionlib server.
	 */
	actionlib::SimpleActionServer <simple_service::MoveToSimpleAction> _server;

	/**
	 * A Listener to read transform information.
	 */
	tf::TransformListener _tf_listener;

	/**
	 * A ros Publisher to publish velocity messages.
	 */
	ros::Publisher _cmd_vel_pub;

	/**
	 * To equate two poses based on a tolerance.
	 */
	bool equals(geometry_msgs::Point a, geometry_msgs::Point b, float tolerance = GOAL_TOLERANCE);

public:
	/**
	 * Called when goal is received.
	 */
	void goalCallback(const simple_service::MoveToSimpleGoalConstPtr& goal);

	/**
	 * Get the current pose.
	 */
	geometry_msgs::PoseStamped getCurrentPose();

	/**
	 * Constructor takes the topic name for command_velocity topic as a std::string.
	 */
	MoveToSimpleServer(std::string cmd_vel_topic_name = "cmd_vel");

	virtual ~MoveToSimpleServer();

};

/**
 * A convenience overloading.
 */
geometry_msgs::Point operator + (geometry_msgs::Point a, geometry_msgs::Point b);

#endif /* MOVE_TO_SIMPLE_SERVER_H_ */
