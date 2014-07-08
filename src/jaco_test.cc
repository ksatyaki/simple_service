#include <unistd.h>
#include <vector>

#include <jaco/CartesianMovementAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>


boost::shared_ptr <tf::TransformListener> _tf_listener;
std::vector<double> _current_pose(6);

void* spin_t(void * dummy)
{
	while(ros::ok())
		ros::spin();

	return NULL;
}

void current_pose_callback(const std_msgs::Float64MultiArrayConstPtr& _msg)
{
	_current_pose = _msg->data;
}

int main(int argn, char *args[])
{
	ros::init(argn, args, "jaco_motion_test");
	ros::NodeHandle nh;

	actionlib::SimpleActionClient <jaco::CartesianMovementAction> C_M_A ("jaco/cartesian_action", true);

	pthread_t id;
	pthread_create(&id, NULL, spin_t, NULL);


	/**
	 * - Translation: [0.146, 0.147, 0.582]
- Rotation: in Quaternion [0.760, 0.412, 0.164, -0.475]
            in RPY [-2.271, -0.694, 0.660]

            At time 1404216280.032
- Translation: [0.211, -0.295, 0.489]
- Rotation: in Quaternion [0.302, -0.628, 0.642, -0.319]
            in RPY [-1.542, 0.013, -2.231]
	 *
	 */

	for(std::vector<double>::iterator i = _current_pose.begin(); i != _current_pose.end(); i++)
	{
		ROS_INFO("%lf", *i);
	}
	ROS_INFO("Is this okay?");
	int i;
	std::cin>>i;

	jaco::CartesianMovementGoal test_goal;
	test_goal.poseGoal.header.frame_id = "hand_jaco";
	test_goal.poseGoal.position.x = -0.05;
	test_goal.poseGoal.position.y = -0.05;
	test_goal.poseGoal.position.z = 0.0;

	test_goal.poseGoal.orientation.x = 0.0;
	test_goal.poseGoal.orientation.y = 0.0;
	test_goal.poseGoal.orientation.z = 0.0;


	ROS_INFO("CHERVER!");
	C_M_A.waitForServer();
	ROS_INFO("WOKAY!");
	C_M_A.sendGoal(test_goal);

	C_M_A.waitForResult();

	ROS_INFO("GOT ITCH!");
}

geometry_msgs::PoseStamped getCurrentPose()
{
	ros::Time _now_stamp_ = ros::Time::now();
	_tf_listener->waitForTransform("jaco_link_6", "base_link", _now_stamp_, ros::Duration(1));

	tf::StampedTransform our_pose_in_tf;
	try
	{
		_tf_listener->lookupTransform("jaco_link_6", "base_link", _now_stamp_, our_pose_in_tf);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("TRANSFORMS ARE COCKED-UP PAL! Why is that :=> %s", ex.what());
	}

	tf::Vector3 our_position = our_pose_in_tf.getOrigin();
	tf::Quaternion our_orientation = our_pose_in_tf.getRotation();

	geometry_msgs::PoseStamped our_pose;
	our_pose.header.stamp = our_pose_in_tf.stamp_;
	our_pose.header.frame_id = our_pose_in_tf.frame_id_;

	tf::pointTFToMsg(our_position, our_pose.pose.position);
	tf::quaternionTFToMsg(our_orientation, our_pose.pose.orientation);

	return our_pose;
}
