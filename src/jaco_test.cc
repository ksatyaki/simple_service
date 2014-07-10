#include <unistd.h>
#include <vector>

#include <jaco/CartesianMovementAction.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco/FingerMovementAction.h>

#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>


int main(int argn, char *args[])
{
	ros::init(argn, args, "jaco_motion_test");
	ros::NodeHandle nh;

	actionlib::SimpleActionClient <jaco::FingerMovementAction> C_M_A ("jaco/finger_action", true);

	jaco::FingerMovementGoal test_goal;
	test_goal.task = jaco::FingerMovementGoal::ABS;
	test_goal.abs_position.push_back(0.1);
	test_goal.abs_position.push_back(0.5);
	test_goal.abs_position.push_back(1.0);

	ROS_INFO("CHERVER!");
	C_M_A.waitForServer();
	ROS_INFO("WOKAY!");
	C_M_A.sendGoal(test_goal);

	C_M_A.waitForResult();

	ROS_INFO("GOT ITCH!");
}
