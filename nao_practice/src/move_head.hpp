#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/JointTrajectoryAction.h>

//namespace naoqi_bridge {
typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointTrajectoryAction> TrajClient;
class RobotHead
{
private:
	TrajClient* traj_client_;

public:
	RobotHead()
	{
		traj_client_ = new TrajClient("joint_trajectory", true);
		while (!traj_client_->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}
	}

	~RobotHead()
	{
		delete traj_client_;
	}

	void startTrajectory(naoqi_bridge_msgs::JointTrajectoryGoal goal)
	{
		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		traj_client_->sendGoal(goal);
	}

	naoqi_bridge_msgs::JointTrajectoryGoal headExtentionTrajectory(double pitch, double yaw)
	{
		naoqi_bridge_msgs::JointTrajectoryGoal goal;

		goal.trajectory.joint_names.push_back("HeadPitch");
		goal.trajectory.joint_names.push_back("HeadYaw");

		goal.trajectory.points.resize(1);

		int ind = 0;
		goal.trajectory.points[ind].positions.resize(2);
		goal.trajectory.points[ind].positions[0] = pitch;
		goal.trajectory.points[ind].positions[1] = yaw;

		goal.trajectory.points[ind].velocities.resize(2);
		for (size_t j = 0; j < 2; j++) {
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}

		goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

		return goal;
	}

	actionlib::SimpleClientGoalState getState()
	{
		return traj_client_->getState();
	}

};
