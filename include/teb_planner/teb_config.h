#ifndef __TEB_CONFIG_H__
#define __TEB_CONFIG_H__
#include <boost/graph/graph_concepts.hpp>
#include <Eigen/Core>

class ConfigStruct
{
public:
	ConfigStruct():min_pose_num_(3),max_pose_num_(100),min_deta_t_(0.1),max_deta_t_(0.3),
				   max_vel_(2),max_start_vel_(0), min_vel_(-1),max_omega_vel_(0),penalty_epsilon_(0.1),
				   max_acc_(0.2),max_omega_acc_(0.05),max_start_omega_vel_(0),max_end_vel_(0),
				   max_end_omega_vel_(0),max_jerk_(0.2),max_omega_jerk_(0.05),min_update_dist_(0.1),
				   max_start_acc_(0.2),max_start_omega_acc_(0.02),max_end_acc_(0.2),max_end_omega_acc_(0.02),
				   min_obstacle_dist_(1)
	{}
	int min_pose_num_;
	int max_pose_num_;
	double min_deta_t_;
	double max_deta_t_;
	
	double max_vel_;
	double max_start_vel_;
	double min_vel_;
	double max_omega_vel_;
	double penalty_epsilon_;
	
	double max_acc_;
	double max_omega_acc_;
	
	double max_start_omega_vel_;
	
	double max_end_vel_;
	double max_end_omega_vel_;
	
	double max_jerk_;
	double max_omega_jerk_;
	
	double min_update_dist_;
	
	double max_start_acc_;
	double max_start_omega_acc_;
	
	double max_end_acc_;
	double max_end_omega_acc_;
	
	double min_obstacle_dist_;
};


class ObstacleStruct
{
public:
	ObstacleStruct()
	{}
	ObstacleStruct(Eigen::Vector2d position)
	{
		position_ = position;
	}
	virtual ~ObstacleStruct()
	{}
	Eigen::Vector2d getPosition(void)
	{
		return position_;
	}
protected:
	Eigen::Vector2d position_;
};


#endif