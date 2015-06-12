#pragma once
 /**
 *
 *  Copyright 2012 Jakob Engel <marcog@unex.es> (University of Extremadura)
 *
 *  ar_lander is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ar_lander is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef __AR_LANDER_H
#define __AR_LANDER_H
 

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "tum_ardrone/filter_state.h"

#include <Eigen/Geometry> 

#include <time.h>

namespace ardrone_tag_landing
{
	class ArLanding
	{
		int old_time, old_dest;
		float old_vel;
		
		int target, landing_target_tag;
		
		bool landing_found;
		
		apriltags_ros::AprilTagDetection previous_detection;
		
		double last_y, last_x, last_z, last_yaw;
		double last_y_land, last_x_land, last_z_land, last_yaw_land;
		
		bool front_camera;
		
		
		ros::Subscriber tag_detections_sub;
		ros::Subscriber tag_for_landing_detections_sub;
		ros::Subscriber target_sub;
		
		ros::Publisher takeoff_pub;
		ros::Publisher land_pub;
		ros::Publisher vel_pub;
		ros::Publisher dronepose_pub;
		ros::Publisher drone_commands_pub;

		ros::Timer change_camera_timer;
		
		ros::ServiceClient toggle_camera_srv;
	public:
		
		ArLanding(ros::NodeHandle& nh);
		~ArLanding();
		
		void process_tag_info(const apriltags_ros::AprilTagDetectionArray& msg);
		void process_tag_landing_info(const apriltags_ros::AprilTagDetectionArray& msg);
		void get_target_info(const std_msgs::Int32& msg);
		void changeCamera(const ros::TimerEvent& event);
		void send_take_off();
		void send_land();
		
	};
}

#endif