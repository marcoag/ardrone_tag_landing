#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


namespace ardrone_tag_landing
{
	class ArLanding
	{
		ros::Subscriber tag_detections_sub;
		public:
			ArLanding(ros::NodeHandle& nh);
			void process_tag_info(const apriltags_ros::AprilTagDetectionArray& msg);
	};
}
