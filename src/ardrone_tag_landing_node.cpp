#include <ardrone_tag_landing/ar_lander.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_tag_landing");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ardrone_tag_landing::ArLanding lander(nh);
	
	ros::spin();
}
