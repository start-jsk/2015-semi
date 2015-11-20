#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "walk_cmd");
	ros::NodeHandle n;
	ros::Publisher walk_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		geometry_msgs::Twist msg;
		msg.linear.x = 0.1;
		msg.linear.y = 0.0;
		msg.linear.z = 0.0;
		
		walk_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
