#include "position_control.h"

const double math_pi = 3.1415926f;

namespace hector_nav
{
	void position_control::set_limit(double *output)
	{
		if(output[0] > pid_xyz[0].limit)       
		{
			output[0] =  pid_xyz[0].limit;
				
		}
		else 
		{
			if(output[0] < -pid_xyz[0].limit) 
			{
				output[0] = -pid_xyz[0].limit;
					
			}
		}
			
		if(output[1] > pid_xyz[1].limit)       
		{
			output[1] =  pid_xyz[1].limit;
		}
		else 
		{
			if(output[1] < -pid_xyz[1].limit) 
			{
				output[1] = -pid_xyz[1].limit;
			}
		}
			
		if(output[2] > pid_xyz[2].limit)       
		{
			output[2] =  pid_xyz[2].limit;
		}
		else 
		{
			if(output[2] < -pid_xyz[2].limit) 
			{
				output[2] = -pid_xyz[2].limit;
			}
		}
		if(output[3] > pid_xyz[3].limit)       
		{
			output[3] =  pid_xyz[3].limit;
		}
		else 
		{
			if(output[3] < -pid_xyz[3].limit) 
			{
				output[3] = -pid_xyz[3].limit;
			}
		}
	}
	double getYaw(const geometry_msgs::Pose &pose)
	{
		const geometry_msgs::Quaternion::_w_type& w = pose.orientation.w;
		const geometry_msgs::Quaternion::_w_type& x = pose.orientation.x;
		const geometry_msgs::Quaternion::_w_type& y = pose.orientation.y;
		const geometry_msgs::Quaternion::_w_type& z = pose.orientation.z;
		return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
	}
	
	void position_control::set_pidxy_parameters(const pid_t& pid)
	{
		this->pid_xyz[0].k_p = pid.k_p;
		this->pid_xyz[0].k_i = pid.k_i;
		this->pid_xyz[0].k_d = pid.k_d;
		this->pid_xyz[0].limit = pid.limit;
		
		this->pid_xyz[1].k_p = pid.k_p;
		this->pid_xyz[1].k_i = pid.k_i;
		this->pid_xyz[1].k_d = pid.k_d;
		this->pid_xyz[1].limit = pid.limit;
	}

	void position_control::set_pidz_parameters(const pid_t& pid)
	{
		this->pid_xyz[2].k_p = pid.k_p;
		this->pid_xyz[2].k_i = pid.k_i;
		this->pid_xyz[2].k_d = pid.k_d;
		this->pid_xyz[2].limit = pid.limit;
	}
	
	void position_control::set_pidyaw_parameters(const pid_t& pid)
	{
		this->pid_xyz[3].k_p = pid.k_p;
		this->pid_xyz[3].k_i = pid.k_i;
		this->pid_xyz[3].k_d = pid.k_d;
		this->pid_xyz[3].limit = pid.limit;
	}
	
	void position_control::update_controller(const nav_msgs::Odometry& command, const nav_msgs::Odometry& state,
	                                         geometry_msgs::Twist& outTwist)
	{
		double deta_t = 0;
		
		double output[4] = {0,0,0,0};
		
		double current_state_x = state.pose.pose.position.x;
		double current_state_y = state.pose.pose.position.y;
		double current_state_z = state.pose.pose.position.z;
		
		double current_command_x = command.pose.pose.position.x;
		double current_command_y = command.pose.pose.position.y;
		double current_command_z = command.pose.pose.position.z;
		
		double current_state_yaw   = getYaw(state.pose.pose);
		double current_command_yaw = command_yaw;
		
		set_current_command(command);
		set_current_state(state);
		
		deta_t = current_state.header.stamp.toSec() - pid_xyz[0].timestamp.toSec();
		
		pid_xyz[0].timestamp = current_state.header.stamp;
		pid_xyz[1].timestamp = current_state.header.stamp;
		pid_xyz[2].timestamp = current_state.header.stamp;
		
		pid_xyz[0].error  =  current_command_x - current_state_x;
		pid_xyz[0].derror =  pid_xyz[0].error  - pid_xyz[0].lerror;
		pid_xyz[0].ierror += pid_xyz[0].error  * deta_t;
		pid_xyz[0].lerror =  pid_xyz[0].error;
		
	    pid_xyz[1].error  =  current_command_y - current_state_y;
		pid_xyz[1].derror =  pid_xyz[1].error  - pid_xyz[1].lerror;
		pid_xyz[1].ierror += pid_xyz[1].error  * deta_t;
		pid_xyz[1].lerror =  pid_xyz[1].error;
		
	    pid_xyz[2].error  =  current_command_z - current_state_z;
		pid_xyz[2].derror =  pid_xyz[2].error  - pid_xyz[2].lerror;
		pid_xyz[2].ierror += pid_xyz[2].error  * deta_t;
		pid_xyz[2].lerror =  pid_xyz[2].error;
		
		if(yaw_control_flag == true)
		{
			pid_xyz[3].error  =  current_command_yaw - current_state_yaw;
			if(pid_xyz[3].error >  math_pi) 
			{
				pid_xyz[3].error -= 2*math_pi;
			}
			else if(pid_xyz[3].error < -math_pi)
			{
					pid_xyz[3].error += 2*math_pi;
			}
			pid_xyz[3].derror =  pid_xyz[3].error  - pid_xyz[3].lerror;
			pid_xyz[3].ierror += pid_xyz[3].error  * deta_t;
			pid_xyz[3].lerror =  pid_xyz[3].error;
		}
		
		output[0] = pid_xyz[0].k_p * pid_xyz[0].error + pid_xyz[0].k_i * pid_xyz[0].ierror
		          + pid_xyz[0].k_d * pid_xyz[0].derror + command.twist.twist.linear.x;
		output[1] = pid_xyz[1].k_p * pid_xyz[1].error + pid_xyz[1].k_i * pid_xyz[1].ierror
		          + pid_xyz[1].k_d * pid_xyz[1].derror + command.twist.twist.linear.y;
	    output[2] = pid_xyz[2].k_p * pid_xyz[2].error + pid_xyz[2].k_i * pid_xyz[2].ierror
				  + pid_xyz[2].k_d * pid_xyz[2].derror + command.twist.twist.linear.z;
		if(yaw_control_flag == true)
		{
			output[3] = pid_xyz[3].k_p * pid_xyz[3].error + pid_xyz[3].k_i * pid_xyz[3].ierror
						+ pid_xyz[3].k_d * pid_xyz[3].derror;
		}
		else
		{
			output[3] = 0;
		}
		set_limit(output);
		
		outTwist.linear.x  =  cos(current_state_yaw) * output[0] + sin(current_state_yaw) * output[1];
		outTwist.linear.y  = -sin(current_state_yaw) * output[0] + cos(current_state_yaw) * output[1];
		outTwist.linear.z  =  output[2];
		outTwist.angular.z =  output[3] + command.twist.twist.angular.z;
	}
	void position_control::stop(geometry_msgs::Twist& outTwist)
	{
		outTwist.linear.x = 0;
		outTwist.linear.y = 0;
		outTwist.linear.z = 0;
		outTwist.angular.z = 0;
	}

}