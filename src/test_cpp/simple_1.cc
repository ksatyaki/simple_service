#include <simple_service/MoveToSimpleAction.h>
#include <actionlib/client/simple_action_client.h>

#include <signal.h>
#include <iostream>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");
	ros::shutdown();

	abort();
}

int main(int argn, char *args[])
{

	// SETUP THE SIGNAL HANDLER //
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);
	// //////////////////////// //

	ros::init(argn, args, "client_simple");
	simple_service::MoveToSimpleGoal a;

	actionlib::SimpleActionClient <simple_service::MoveToSimpleAction> simple_client("move_to_simple", true);

	ROS_INFO("WAITING FOR SERVER!");
	simple_client.waitForServer();
	ROS_INFO("Found. Sending goal.");

	while(ros::ok())
	{
		std::cin>>a.goal_pose.pose.position.x;
		std::cin>>a.goal_pose.pose.position.y;
		std::cin>>a.goal_pose.pose.orientation.z;
		std::cin>>a.goal_pose.pose.orientation.w;

		std::cin>>a.driving_direction;

		a.xy_tolerance = 0.1;
		a.yaw_tolerance = 0.1;

		simple_client.sendGoal(a);

		simple_client.waitForResult();

		ROS_INFO("GOT IT...");
	}

	return 0;
}

