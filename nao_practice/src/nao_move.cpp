#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointTrajectoryAction> TrajClient;

class RobotArm
{
private:
	TrajClient* traj_client_;

public:
	RobotArm()
	{
		traj_client_ = new TrajClient("joint_trajectory", true);
		while (!traj_client_->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}
	}

	~RobotArm()
	{
		delete traj_client_;
	}

	void startTrajectory(naoqi_bridge_msgs::JointTrajectoryGoal goal)
	{
		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		traj_client_->sendGoal(goal);
	}

	naoqi_bridge_msgs::JointTrajectoryGoal armExtentionTrajectory()
	{
		naoqi_bridge_msgs::JointTrajectoryGoal goal;

		goal.trajectory.joint_names.push_back("LShoulderPitch");
		goal.trajectory.joint_names.push_back("LShoulderRoll");
		goal.trajectory.joint_names.push_back("LElbowYaw");
		goal.trajectory.joint_names.push_back("LElbowRoll");

		goal.trajectory.points.resize(2);

		int ind = 0;
		goal.trajectory.points[ind].positions.resize(4);
		goal.trajectory.points[ind].positions[0] = 0.0;
		goal.trajectory.points[ind].positions[1] = 0.0;
		goal.trajectory.points[ind].positions[2] = 0.0;
		goal.trajectory.points[ind].positions[3] = 0.0;

		goal.trajectory.points[ind].velocities.resize(4);
		for (size_t j = 0; j < 4; j++) {
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}

		goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

		ind++;

		goal.trajectory.points[ind].positions.resize(4);
		goal.trajectory.points[ind].positions[0] = 1.0;
		goal.trajectory.points[ind].positions[1] = 1.0;
		goal.trajectory.points[ind].positions[2] = 1.0;
		goal.trajectory.points[ind].positions[3] = -1.0;

		goal.trajectory.points[ind].velocities.resize(4);
		for (size_t j = 0; j < 4; j++) {
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}

		goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

		return goal;
	}

	actionlib::SimpleClientGoalState getState()
	{
		return traj_client_->getState();
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_driver");

	RobotArm arm;

	arm.startTrajectory(arm.armExtentionTrajectory());
	while (!arm.getState().isDone() && ros::ok()) {
		usleep(50000);
	}
}
