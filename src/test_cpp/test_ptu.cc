#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

#include <ptu_control/PtuGotoAction.h>
#include <math.h>

ros::Publisher pub_ptu_;

boost::shared_ptr<sensor_msgs::JointState> current_state;

void ptuStateCallback(const sensor_msgs::JointStateConstPtr& _msg)
{
	current_state = boost::shared_ptr<sensor_msgs::JointState> (new sensor_msgs::JointState);
	*current_state = *_msg;
}

bool PanTiltToPoint(const geometry_msgs::PointStamped& Pt)
{
	double reqd_tilt = asin((1.42 - Pt.point.z)/ sqrt((Pt.point.x*Pt.point.x) + (Pt.point.y*Pt.point.y)));
	double reqd_pan = asin(Pt.point.y/ sqrt((Pt.point.x*Pt.point.x) + (Pt.point.y*Pt.point.y)));

	ROS_INFO("PTU IS BEING PANNED and TILTED!");
	sensor_msgs::JointState ptu_msg;
	ptu_msg.header.stamp = ros::Time::now() + ros::Duration(0.01);

	ptu_msg.name.resize(2);
	ptu_msg.position.resize(2);
	ptu_msg.velocity.resize(2);
	ptu_msg.effort.resize(2);

	ptu_msg.name[0] = "ptu_pan_joint";
	ptu_msg.name[1] = "ptu_tilt_joint";

	ROS_INFO("Required pan = %lf", reqd_pan);
	ROS_INFO("Required tilt = %lf", reqd_tilt);
	ptu_msg.position[0]= reqd_pan;
	ptu_msg.position[1]= -1 * reqd_tilt;

	ptu_msg.velocity[0]=0.5;
	ptu_msg.velocity[1]=0.5;

	sleep(1);
	pub_ptu_.publish(ptu_msg);
	sleep(1);
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "test_ptu_node");
	ros::NodeHandle nh;
	ros::CallbackQueue q;
	nh.setCallbackQueue(&q);

	//ros::Subscriber js_sub = nh.subscribe("ptu/joint_states", 1, ptuStateCallback);

	pub_ptu_ = nh.advertise<sensor_msgs::JointState>("/ptu/cmd", 1);

	geometry_msgs::PointStamped view_pt;

	view_pt.point.x = 1.0;
	view_pt.point.y = 0.0;
	view_pt.point.z = 1.42;

	//while(!current_state)
	//{
		//q.callOne(ros::WallDuration(1.0));
		//ROS_INFO("We waits.");
		//sleep(1);
	//}
	//ROS_INFO("Got the state.");

	PanTiltToPoint(view_pt);

	return 0;

}
