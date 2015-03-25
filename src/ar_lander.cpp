#include <ardrone_tag_landing/ar_lander.h>

namespace ardrone_tag_landing
{
 	ArLanding::ArLanding(ros::NodeHandle& nh)
	{
 		tag_detections_sub = nh.subscribe("tag_detections", 1000, &ArLanding::process_tag_info, this);
 	}
 	
 	void ArLanding::process_tag_info(const apriltags_ros::AprilTagDetectionArray& msg)
	{
		for (std::vector<apriltags_ros::AprilTagDetection>::const_iterator it = msg.detections.begin() ; it != msg.detections.end(); ++it)
		{
			std::cout<<it->id<<std::endl;
		}
	}
}