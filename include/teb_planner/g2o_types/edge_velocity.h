#ifndef __EDGE_VELOCITY_H__
#define __EDGE_VELOCITY_H__

#include <g2o/core/base_multi_edge.h>
#include <cmath>

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"
#include "g2o_types/base_teb_edges.h"
#include "g2o_types/penalties.h"

#include "teb_config.h"

class EdgeVelocity : public g2o::BaseMultiEdge<2, double>
{
public:
	EdgeVelocity()
	{
		this->resize(3);
	}
	virtual ~EdgeVelocity()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}

	void computeError(void)
	{
		const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
		
		//计算
		Eigen::Vector2d detaS = conf2->estimate().position() - conf1->estimate().position();
		double dist = detaS.norm();
		
		double turning_omega = conf2->estimate().theta() - conf1->estimate().theta();
		//机体的转弯半径
		if(turning_omega != 0)
		{
			double radius = dist/(2*sin(turning_omega/2));
			//实际机体的行走距离
			dist = fabs(radius*turning_omega);
		}

		double vel = dist/deltaT->estimate();
		vel *= g2o::sign(detaS[0]*cos(conf1->theta()) + detaS[1]*sin(conf1->theta()));
		double omega_vel = turning_omega/deltaT->estimate();

		//代价函数
		_error[0] = penaltyBoundToInterval(vel, config_.min_vel_, config_.max_vel_, config_.penalty_epsilon_);
		_error[1] = penaltyBoundToInterval(omega_vel, config_.max_omega_vel_,0);
		
	}
	
	void setConfig(ConfigStruct& config)
	{
		config_ = config;
	}
	
	bool read(std::istream& is)
	{
		return true;
	}
	bool write(std::ostream& os) const
	{
		return os.good();
	}
protected:
	ConfigStruct config_;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
};

class EdgeStartVelocity : public g2o::BaseMultiEdge<2, double>
{
public:
	EdgeStartVelocity()
	{
		this->resize(3);
	}
	virtual ~EdgeStartVelocity()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}

	void computeError(void)
	{
		const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
		
		//计算
		Eigen::Vector2d detaS = conf2->estimate().position() - conf1->estimate().position();
		double dist = detaS.norm();
		
		double turning_omega = conf2->estimate().theta() - conf1->estimate().theta();
		//机体的转弯半径
		if(turning_omega != 0)
		{
			double radius = dist/(2*sin(turning_omega/2));
			//实际机体的行走距离
			dist = fabs(radius*turning_omega);
		}

		double vel = dist/deltaT->estimate();
		vel *= g2o::sign(detaS[0]*cos(conf1->theta()) + detaS[1]*sin(conf1->theta()));
		double omega_vel = turning_omega/deltaT->estimate();

		//代价函数
		_error[0] = penaltyBoundToInterval(vel, config_.max_start_vel_, config_.penalty_epsilon_);;
		_error[1] = penaltyBoundToInterval(omega_vel, config_.max_start_omega_vel_,0);
	}
	
	void setConfig(ConfigStruct& config)
	{
		config_ = config;
	}
	
	bool read(std::istream& is)
	{
		return true;
	}
	bool write(std::ostream& os) const
	{
		return os.good();
	}
protected:
	ConfigStruct config_;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeEndVelocity : public g2o::BaseMultiEdge<2, double>
{
public:
	EdgeEndVelocity()
	{
		this->resize(3);
	}
	virtual ~EdgeEndVelocity()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}

	void computeError(void)
	{
		const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
		
		//计算
		Eigen::Vector2d detaS = conf2->estimate().position() - conf1->estimate().position();
		double dist = detaS.norm();
		
		double turning_omega = conf2->estimate().theta() - conf1->estimate().theta();
		//机体的转弯半径
		if(turning_omega != 0)
		{
			double radius = dist/(2*sin(turning_omega/2));
			//实际机体的行走距离
			dist = fabs(radius*turning_omega);
		}

		double vel = dist/deltaT->estimate();
		vel *= g2o::sign(detaS[0]*cos(conf1->theta()) + detaS[1]*sin(conf1->theta()));
		double omega_vel = turning_omega/deltaT->estimate();

		//代价函数
		_error[0] = penaltyBoundToInterval(vel, config_.max_end_vel_, config_.penalty_epsilon_);;
		_error[1] = penaltyBoundToInterval(omega_vel, config_.max_end_omega_vel_,0);
	}
	
	void setConfig(ConfigStruct& config)
	{
		config_ = config;
	}
	
	bool read(std::istream& is)
	{
		return true;
	}
	bool write(std::ostream& os) const
	{
		return os.good();
	}
protected:
	ConfigStruct config_;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif