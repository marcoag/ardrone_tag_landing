#include <ardrone_tag_landing/ar_lander.h>

namespace ardrone_tag_landing
{
 	ArLanding::ArLanding(ros::NodeHandle& nh): target(-1), landing_target_tag(-1), landing_found(false), front_camera(true)
	{
		//suscribers
 		tag_detections_sub = nh.subscribe(nh.resolveName("/apriltag_detector_front/tag_detections"), 1000, &ArLanding::process_tag_info, this);
		tag_for_landing_detections_sub = nh.subscribe(nh.resolveName("/apriltag_detector_bottom/tag_detections"), 1000, &ArLanding::process_tag_landing_info, this);
		target_sub = nh.subscribe(nh.resolveName("/ardrone_target"), 1000, &ArLanding::get_target_info, this);
		
		//publishers
		takeoff_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/takeoff"),1);
		vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
		dronepose_pub = nh.advertise<tum_ardrone::filter_state>(nh.resolveName("ardrone/predictedPose"),1);
		land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
		drone_commands_pub = nh.advertise<std_msgs::String>(nh.resolveName("/tum_ardrone/com"),1);
		 
		//timer for change camera
		change_camera_timer = nh.createTimer(ros::Duration(0.5),  &ArLanding::changeCamera, this);
		
		//toggle camera service call
		toggle_camera_srv = nh.serviceClient<std_srvs::Empty>(nh.resolveName("ardrone/togglecam"),1);
		
		last_x = last_y = last_z = last_yaw = 0;
		last_x_land = last_y_land = last_z_land = last_yaw_land = 0;
		
 	}
 	
 	 ArLanding::~ArLanding()
	{
		send_land();
 	}
 	
 	void ArLanding::send_take_off()
	{
		
		takeoff_pub.publish(std_msgs::Empty());
		ROS_INFO_STREAM("Taking off, they see me rollin!");
		ros::spinOnce();
	}
	
	void ArLanding::send_land()
	{
		
		land_pub.publish(std_msgs::Empty());
		ROS_INFO_STREAM("Lainding, they hatin");
		ros::spinOnce();
	}
	
// 	void ArLanding::get_velocity(float new_dist)
// 	{
// 		
// 		int new_time = time(NULL);
// 		
// 		//velocity = distance/time
// 		float vel = (new_dist - old_dist)/(new_time - old_time);
//  
// 		// smooth velocity
// 		old_vel = old_vel + (vel - old_vel) * 0.1;
//  
// 		// save values for next time
// 		old_time = new_time;
// 		old_dist = new_dist;
//  
// 		return old_vel;
// 	}

	void ArLanding::changeCamera(const ros::TimerEvent& event)
	{
		std::cout<<"getting callbacked"<<std::endl;
		std_srvs::Empty toggleCam_srv_srvs;
		toggle_camera_srv.call(toggleCam_srv_srvs);
		front_camera = !front_camera;
		return;
	}

 	void ArLanding::get_target_info(const std_msgs::Int32& msg)
	{
		if (msg.data == 0 || msg.data == 2 || msg.data == 4)
		{
			target = msg.data;
			if(msg.data == 0)
				landing_target_tag = 1;
			if(msg.data == 2)
				landing_target_tag = 2;
			if(msg.data == 4)
				landing_target_tag = 5;
		}
		else
			target = -1;
	}
	
	void ArLanding::process_tag_landing_info(const apriltags_ros::AprilTagDetectionArray& msg)
	{
		landing_found = false;
		
		tum_ardrone::filter_state s;
		s.header.stamp = ros::Time().now();
		s.droneState = 3; //flying
		s.batteryPercent = 90;
		s.ptamState == s.PTAM_BEST;
		s.scaleAccuracy = 1;
		s.scale = 1;
		Eigen::Quaternion<double> q;
		
		if(landing_target_tag != -1 && !front_camera)
		{
			for (std::vector<apriltags_ros::AprilTagDetection>::const_iterator it = msg.detections.begin() ; it != msg.detections.end(); ++it)
			{
				ROS_INFO_STREAM("BOTTOM_ID: "<<it->id<<" X: "<<it->pose.pose.position.x<<", Y: "<<it->pose.pose.position.y<<", Z:"<<it->pose.pose.position.z);
				
				if(it->id == landing_target_tag)
				{
					landing_found = true;
					std::cout<<"LANDING FOUND!!"<<std::endl;
	
					//if too close to the floor only adjust pitch
					if (it->pose.pose.position.z < 1)
					{
						s.z = 0;
						std_msgs::String command;
						command.data = "c land";
						drone_commands_pub.publish(command);
						command.data = "c stop";
						drone_commands_pub.publish(command);
						target = -1;
					}
					else
						s.z = it->pose.pose.position.z;
					
					
					s.x = it->pose.pose.position.y;
					s.y = it->pose.pose.position.x;
					
					
					q = Eigen::Quaternion<double>(it->pose.pose.orientation.w, it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z);
					
					s.dx = last_x_land - s.x;
					s.dy = last_y_land - s.y;
					s.dz = last_z_land - s.z;
					
					last_x_land = s.x;
					last_y_land = s.y;
					last_z_land = s.z;
					
					Eigen::Matrix3d rot = q.toRotationMatrix();
						
					Eigen::Matrix<double, 3, 1> euler = rot.eulerAngles(0, 1, 2);
					
					double yaw = euler(0,0);
					double roll = euler(1,0);
					double pitch = euler(2,0);
					
					ROS_INFO_STREAM("Yaw: "<<yaw<<", Roll: "<<roll<<", Pitch: "<<pitch);
					
					//we use roll of the mark, since we have it in vertical mode
					s.yaw = roll;
					s.dyaw = last_yaw_land - roll;
					
					last_yaw_land = s.yaw;
					
					dronepose_pub.publish(s);
				}
			}
		}
	}
	
 	void ArLanding::process_tag_info(const apriltags_ros::AprilTagDetectionArray& msg)
	{
		bool tag_found = false;
		
		tum_ardrone::filter_state s;
		s.header.stamp = ros::Time().now();
		s.droneState = 3; //flying
		s.batteryPercent = 90;
		s.ptamState == s.PTAM_BEST;
		s.scaleAccuracy = 1;
		s.scale = 1;
		Eigen::Quaternion<double> q;
		double trans_x, trans_y, trans_z;
		
		if(!landing_found && front_camera)
		{
			if( target != -1)
			{	
				for (std::vector<apriltags_ros::AprilTagDetection>::const_iterator it = msg.detections.begin() ; it != msg.detections.end(); ++it)
				{
					ROS_INFO_STREAM("ID: "<<it->id<<" X: "<<it->pose.pose.position.x<<", Y: "<<it->pose.pose.position.y<<", Z:"<<it->pose.pose.position.z);
						//if id is the same as target
					if( it->id == 0 )
					{
						
						
						if(target == 0)
						{
							tag_found = true;
							s.x = -it->pose.pose.position.x;
							s.y = -it->pose.pose.position.z;
							s.z = it->pose.pose.position.y;
						}
						
	// 					if(target == 2)
	// 					{
	// 						std::cout<<"id 0 target 2"<<std::endl;
	// 						s.x = -it->pose.pose.position.x + 2.81;
	// 						s.y = -it->pose.pose.position.z + 5;
	// 						s.z = it->pose.pose.position.y - 0.09;
	// 					}
	// 					
	// 					if(target == 4)
	// 					{
	// 						std::cout<<"id 0 target 4"<<std::endl;
	// 						s.x = -it->pose.pose.position.x + 2.41;
	// 						s.y = -it->pose.pose.position.z - 5.69;
	// 						s.z = it->pose.pose.position.y - 0.041;
	// 					}
						
						q = Eigen::Quaternion<double>(it->pose.pose.orientation.w, it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z);
						
					}
					if( it->id == 3 )
					{
						if(target == 2)
						{
							tag_found = true;
							s.x = -it->pose.pose.position.x;
							s.y = -it->pose.pose.position.z;
							s.z = it->pose.pose.position.y;
						}
						
						q = Eigen::Quaternion<double>(it->pose.pose.orientation.w, it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z);
					}
					
					if( it->id == 4 )
					{
						
						if(target == 4)
						{
							tag_found = true;
							s.x = -it->pose.pose.position.x;
							s.y = -it->pose.pose.position.z;
							s.z = it->pose.pose.position.y;
						}
						
						q = Eigen::Quaternion<double>(it->pose.pose.orientation.w, it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z);
					}
		
					
					s.dx = last_x - s.x;
					s.dy = last_y - s.y;
					s.dz = last_z - s.z;
					
					last_x = s.x;
					last_y = s.y;
					last_z = s.z;
					
					Eigen::Matrix3d rot = q.toRotationMatrix();
						
					Eigen::Matrix<double, 3, 1> euler = rot.eulerAngles(0, 1, 2);
					
					double yaw = euler(0,0);
					double roll = euler(1,0);
					double pitch = euler(2,0);
					
					ROS_INFO_STREAM("Yaw: "<<yaw<<", Roll: "<<roll<<", Pitch: "<<pitch);
					
					//we use pitch of the mark, since we have it in vertical mode
					s.yaw = pitch;
					s.dyaw = last_yaw - pitch;
					
					last_yaw = s.yaw;
					
					dronepose_pub.publish(s);
				}
			}
			
			if( !tag_found )
			{
				ROS_INFO_STREAM("No tag");
				tum_ardrone::filter_state s;
				s.header.stamp = ros::Time().now();
				s.droneState = 3; //flying
				s.batteryPercent = 90;

				s.x = 0;
				s.y = 0;
				s.z = 0;

				s.dx = 0;
				s.dy = 0;
				s.dz = 0;

				s.roll = 0;
				s.pitch = 0;
				
				//if target not initiated hover
				//otherwise rotate to find markers
				if (target == -1)
					s.yaw=0;
				else
				{
					std::cout<<"Rollin"<<std::endl;
					s.yaw = -1;
				}
				
				s.dyaw = 0;

				s.ptamState == s.PTAM_BEST;
				//this is a very bad hack, don't try this at home
				s.scaleAccuracy = 3;
				s.scale = 1;
				
				dronepose_pub.publish(s);
			}
		}
	}
}