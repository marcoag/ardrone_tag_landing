#include <ardrone_tag_landing/ar_lander.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_tag_landing");
	ros::NodeHandle nh;
	ardrone_tag_landing::ArLanding lander(nh);
	ros::spin();
}
