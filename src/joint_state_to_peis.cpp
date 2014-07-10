/*
 * joint_state_to_peis.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Chittaranjan S Srinivas
 *
 * This node reads the jaco/joint_states topic and publishes the information on PEIS network.
 * This node also reads the jaco/current_pose and publishes the information on PEIS.
 */

#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

extern "C"
{
	#include <peiskernel/peiskernel.h>
	#include <peiskernel/peiskernel_mt.h>
}

class JointStateToPeis
{
	/**
	 * NodeHandle.
	 */
	ros::NodeHandle nh_;

	/**
	 * A thread id for the current_pose publisher thread.
	 */
	pthread_t current_pose_publisher_thread_id_;

	/**
	 * A thread id for the joint_states publisher thread
	 */
	pthread_t joint_state_publisher_thread_id_;

	/**
	 * A subscriber to the 'jaco/joint_states' topic.
	 */
	ros::Subscriber joint_state_sub_;

	/**
	 * A Subscriber to the 'jaco/current_pose' topic.
	 */
	ros::Subscriber current_pose_sub_;

	/**
	 * A variable to collect the latest joint_states message.
	 */
	sensor_msgs::JointState joint_state_msg_;

	/**
	 * A variable to collect the latest current_pose message.
	 */
	std_msgs::Float64MultiArray current_pose_msg_;

	/**
	 * Publish rate for the current_pose tuples.
	 * ros::Rate is very easy to use.
	 */
	boost::shared_ptr<ros::Rate> current_pose_pub_freq_;

	/**
	 * Publish rate for the joint_states PEIS tuples.
	 * ros::Rate is very easy to use.
	 */
	boost::shared_ptr<ros::Rate> joint_state_pub_freq_;

	/**
	 * A thread for publishing current_pose information on PEIS.
	 */
	static void *current_pose_publisher_thread(void *object);

	/**
	 * A thread for publishing joint_state information on PEIS.
	 */
	static void *joint_state_publisher_thread(void *object);

	/**
	 * A callback for the joint_state_sub_;
	 */
	void jointStateCB(const sensor_msgs::JointStateConstPtr& _msg);

	/**
	 * A callback for the current_pose_sub_;
	 */
	void currentPoseCB(const std_msgs::Float64MultiArrayConstPtr& _msg);

	/**
	 * A callback for publishing frequency of joint_states.
	 */
	static void jointStatePubFreqCB(PeisTuple *p, void *dummy);

	/**
	 * A callback for publishing frequency of joint_states.
	 */
	static void currentPosePubFreqCB(PeisTuple *p, void *dummy);

public:

	/**
	 * Constructor.
	 */
	JointStateToPeis();

	/**
	 * Destructor.
	 */
	virtual ~JointStateToPeis();
};

JointStateToPeis::JointStateToPeis()
{
	int my_peis_id = peiskmt_peisid();
	joint_state_sub_ = nh_.subscribe("jaco/joint_states", 1, &JointStateToPeis::jointStateCB, this);
	current_pose_sub_ = nh_.subscribe("jaco/current_pose", 1, &JointStateToPeis::currentPoseCB, this);

	peiskmt_registerTupleCallback(my_peis_id, "jaco_joint_states.publishing_frequency", this, &JointStateToPeis::jointStatePubFreqCB);
	peiskmt_registerTupleCallback(my_peis_id, "jaco_current_pose.publishing_frequency", this, &JointStateToPeis::currentPosePubFreqCB);

	joint_state_pub_freq_ = boost::shared_ptr<ros::Rate> (new ros::Rate(10.0));
	joint_state_pub_freq_ = boost::shared_ptr<ros::Rate> (new ros::Rate(10.0));

	peiskmt_setStringTuple("jaco_joint_states.publishing_frequency", "10.00");
	peiskmt_setStringTuple("jaco_current_pose.publishing_frequency", "10.00");

	peiskmt_setStringTuple("jaco_joint_states.format", "j0 j1 j2 j3 j4 j5 j6");
	peiskmt_setStringTuple("jaco_current_pose.format", "x y z pitch yaw roll");

	pthread_create(&current_pose_publisher_thread_id_, NULL, &JointStateToPeis::current_pose_publisher_thread, (void *) this);
	pthread_create(&joint_state_publisher_thread_id_, NULL, &JointStateToPeis::joint_state_publisher_thread, (void *) this);

	ROS_INFO("Object created!");

}

JointStateToPeis::~JointStateToPeis()
{
	pthread_cancel(joint_state_publisher_thread_id_);
	pthread_cancel(current_pose_publisher_thread_id_);
}

void JointStateToPeis::jointStateCB(const sensor_msgs::JointStateConstPtr& _msg)
{
	joint_state_msg_ = *_msg;
}


void JointStateToPeis::currentPoseCB(const std_msgs::Float64MultiArrayConstPtr& _msg)
{
	current_pose_msg_ = *_msg;
}

void JointStateToPeis::jointStatePubFreqCB(PeisTuple *p, void *object)
{
	float publish_frequency;
	sscanf(p->data, "%f", &publish_frequency);
	((JointStateToPeis*) object)->joint_state_pub_freq_.reset();
	((JointStateToPeis*) object)->joint_state_pub_freq_ = boost::shared_ptr<ros::Rate> (new ros::Rate(publish_frequency));
}

void JointStateToPeis::currentPosePubFreqCB(PeisTuple *p, void *object)
{
	float publish_frequency;
	sscanf(p->data, "%f", &publish_frequency);
	((JointStateToPeis*) object)->current_pose_pub_freq_.reset();
	((JointStateToPeis*) object)->current_pose_pub_freq_ = boost::shared_ptr<ros::Rate> (new ros::Rate(publish_frequency));
}

void* JointStateToPeis::joint_state_publisher_thread(void *object)
{

	char *joint_effort_str = new char[50];
	char *joint_position_str = new char[50];

	char *finger_position_str = new char[30];
	char *finger_effort_str = new char[30];

	ROS_INFO("Started publishing joint states to PEIS...");

	while(ros::ok() && peisk_isRunning())
	{
		if(((JointStateToPeis*) object)->joint_state_msg_.position.size() != 9)
		{
			strcpy(joint_position_str, "0.0 0.0 0.0 0.0 0.0 0.0");
			strcpy(finger_position_str, "0.0 0.0 0.0");
		}
		else
		{
			sprintf(joint_position_str, "%0.4lf %0.4lf %0.4lf %0.4lf %0.4lf %0.4lf",
				((JointStateToPeis*) object)->joint_state_msg_.position[0], ((JointStateToPeis*) object)->joint_state_msg_.position[1],
				((JointStateToPeis*) object)->joint_state_msg_.position[2], ((JointStateToPeis*) object)->	joint_state_msg_.position[3],
				((JointStateToPeis*) object)->joint_state_msg_.position[4], ((JointStateToPeis*) object)->joint_state_msg_.position[5]);

			sprintf(finger_position_str, "%0.4lf %0.4lf %0.4lf",
					((JointStateToPeis*) object)->joint_state_msg_.position[6],
					((JointStateToPeis*) object)->joint_state_msg_.position[7],
					((JointStateToPeis*) object)->joint_state_msg_.position[8]);
		}

		if(((JointStateToPeis*) object)->joint_state_msg_.effort.size() != 9)
		{
			strcpy(joint_effort_str, "0.0 0.0 0.0 0.0 0.0 0.0");
			strcpy(finger_effort_str, "0.0 0.0 0.0");
		}
		else
		{
			sprintf(joint_effort_str, "%0.4lf %0.4lf %0.4lf %0.4lf %0.4lf %0.4lf",
				((JointStateToPeis*) object)->joint_state_msg_.effort[0], ((JointStateToPeis*) object)->joint_state_msg_.effort[1],
				((JointStateToPeis*) object)->joint_state_msg_.effort[2], ((JointStateToPeis*) object)->joint_state_msg_.effort[3],
				((JointStateToPeis*) object)->joint_state_msg_.effort[4], ((JointStateToPeis*) object)->joint_state_msg_.effort[5]);

			sprintf(finger_effort_str, "%0.4lf %0.4lf %0.4lf",
					((JointStateToPeis*) object)->joint_state_msg_.effort[6],
					((JointStateToPeis*) object)->joint_state_msg_.effort[7],
					((JointStateToPeis*) object)->joint_state_msg_.effort[8]);
		}

		peiskmt_setStringTuple("jaco_joint_states.position", joint_position_str);
		peiskmt_setStringTuple("jaco_joint_states.effort", joint_effort_str);
		peiskmt_setStringTuple("jaco_finger.position", finger_position_str);
		peiskmt_setStringTuple("jaco_finger.effort", finger_effort_str);

		(((JointStateToPeis*) object)->joint_state_pub_freq_)->sleep();
	}

	delete joint_effort_str;
	delete joint_position_str;

	delete finger_position_str;
	delete finger_effort_str;

	return NULL;
}

void* JointStateToPeis::current_pose_publisher_thread(void *object)
{
	char *current_pose_str = new char[50];

	ROS_INFO("Started publishing current pose to PEIS...");

	while(ros::ok() && peisk_isRunning())
	{

		if(((JointStateToPeis*) object)->current_pose_msg_.data.size() != 6)
			strcpy(current_pose_str, "0.0 0.0 0.0 0.0 0.0 0.0");
		else
			sprintf(current_pose_str, "%0.4lf %0.4lf %0.4lf %0.4lf %0.4lf %0.4lf",
				((JointStateToPeis*) object)->current_pose_msg_.data[0], ((JointStateToPeis*) object)->current_pose_msg_.data[1],
				((JointStateToPeis*) object)->current_pose_msg_.data[2], ((JointStateToPeis*) object)->current_pose_msg_.data[3],
				((JointStateToPeis*) object)->current_pose_msg_.data[4], ((JointStateToPeis*) object)->current_pose_msg_.data[5]);

		peiskmt_setStringTuple("jaco_current_pose.pose", current_pose_str);

		(((JointStateToPeis*) object)->current_pose_pub_freq_)->sleep();
	}

	delete current_pose_str;
	return NULL;
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "joint_state_to_peis");
	peiskmt_initialize(&argn, args);

	JointStateToPeis _my_pub_;

	ROS_INFO("Main thread spinning...");

	ros::spin();

	return 0;
}



