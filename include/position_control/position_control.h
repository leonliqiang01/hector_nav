#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

namespace hector_nav
{
	class pid_t
	{
	public:
		double k_p;
		double k_i;
		double k_d;
				
		double error;
		double lerror;
		double ierror;
		double derror;

		double limit;
				
		ros::Time timestamp;
				
		pid_t():k_p(1.5),k_i(0),k_d(0),error(0),lerror(0),ierror(0),derror(0)
		{}
		virtual ~pid_t(){}
	};
	class position_control
	{
	public:
		position_control()
		{}
		virtual ~position_control()
		{}

		void set_pidxy_parameters(const pid_t &pid);
		void set_pidz_parameters(const pid_t &pid);
		void set_pidyaw_parameters(const pid_t &pid);
		void update_controller(const nav_msgs::Odometry &command, const nav_msgs::Odometry &state, geometry_msgs::Twist &outTwist);
		void stop(geometry_msgs::Twist &outTwist);
		pid_t  pid_xyz[4];
		double command_yaw;
		bool   yaw_control_flag;
		
	private:
		void set_current_command(const nav_msgs::Odometry &command)
		{
			this->current_command = command;
		}
		void set_current_state(const nav_msgs::Odometry &state)
		{
			this->current_state = state;
		}
		void set_limit(double *output);
		
		
		nav_msgs::Odometry current_state;
		nav_msgs::Odometry current_command; 
	};
}

#endif