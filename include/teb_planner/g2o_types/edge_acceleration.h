#ifndef __EDGE_ACCELERATION_H__
#define __EDGE_ACCELERATION_H__

#include <g2o/core/base_multi_edge.h>
#include <cmath>

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"
#include "g2o_types/penalties.h"
#include "g2o_types/base_teb_edges.h"

#include "teb_config.h"

class EdgeAcceleration : public g2o::BaseMultiEdge<2, double>
{
public:

	EdgeAcceleration()
	{
		this->resize(5);
	}
	
	virtual ~EdgeAcceleration()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}
	void computeError(void)
	{
		const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
		const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
		const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

		// VELOCITY & ACCELERATION
		const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
		const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
	
		double dist1 = diff1.norm();
		double dist2 = diff2.norm();
		const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
		const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
		
		if (angle_diff1 != 0)
		{
			const double radius =  dist1/(2*sin(angle_diff1/2));
			dist1 = fabs( angle_diff1 * radius ); // actual arg length!
		}
		if (angle_diff2 != 0)
		{
			const double radius =  dist2/(2*sin(angle_diff2/2));
			dist2 = fabs( angle_diff2 * radius ); // actual arg length!
		}
		
		double vel1 = dist1 / dt1->dt();
		double vel2 = dist2 / dt2->dt();
		
		
		// consider directions
		vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
		vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta())); 

		const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );

		_error[0] = penaltyBoundToInterval(acc_lin,config_.max_acc_,config_.penalty_epsilon_);
		
		// ANGULAR ACCELERATION
		const double omega1 = angle_diff1 / dt1->dt();
		const double omega2 = angle_diff2 / dt2->dt();
		const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
		
		_error[1] = penaltyBoundToInterval(acc_rot,config_.max_omega_acc_,0);
	}
	void setConfig(const ConfigStruct& config)
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

class EdgeJerk : public g2o::BaseMultiEdge<2, double>
{
public:

	EdgeJerk()
	{
		this->resize(7);
	}
	
	virtual ~EdgeJerk()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}
	void computeError(void)
	{
		const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
		const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
		const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
		const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
		const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

		// VELOCITY & ACCELERATION
		const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
		const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
		const Eigen::Vector2d diff3 = pose4->position() - pose3->position();
	
		double dist1 = diff1.norm();
		double dist2 = diff2.norm();
		double dist3 = diff3.norm();
		const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
		const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
		const double angle_diff3 = g2o::normalize_theta(pose4->theta() - pose3->theta());
		
		if (angle_diff1 != 0)
		{
			const double radius =  dist1/(2*sin(angle_diff1/2));
			dist1 = fabs( angle_diff1 * radius ); // actual arg length!
		}
		if (angle_diff2 != 0)
		{
			const double radius =  dist2/(2*sin(angle_diff2/2));
			dist2 = fabs( angle_diff2 * radius ); // actual arg length!
		}
		if(angle_diff3 != 0)
		{
			const double radius =  dist3/(2*sin(angle_diff3/2));
			dist3 = fabs( angle_diff3 * radius ); // actual arg length!
		}
		
		double vel1 = dist1 / dt1->dt();
		double vel2 = dist2 / dt2->dt();
		double vel3 = dist3 / dt3->dt();
		
		// consider directions
		vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
		vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta())); 
		vel3 *= g2o::sign(diff3[0]*cos(pose3->theta()) + diff3[1]*sin(pose3->theta()));
		
		const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );
		const double acc_lin2 = (vel3 - vel2)*2 / ( dt2->dt() + dt3->dt() );
		
		const double acc_jerk = (acc_lin2 - acc_lin)*3 / ( dt1->dt() + dt2->dt() + dt3->dt() ); 

// 		_error[0] = penaltyBoundToInterval(acc_jerk,config_.max_jerk_,0);
		_error[0] = acc_jerk;
		
		// ANGULAR ACCELERATION
		const double omega1 = angle_diff1 / dt1->dt();
		const double omega2 = angle_diff2 / dt2->dt();
		const double omega3 = angle_diff3 / dt3->dt();
		const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
		const double acc_rot2 = (omega3 - omega2)*2 / ( dt2->dt() + dt3->dt() );
		
		const double rot_jerk = (acc_rot2 - acc_rot)*3 / ( dt1->dt() + dt2->dt() + dt3->dt() );
		
// 		_error[1] = penaltyBoundToInterval(rot_jerk,config_.max_omega_jerk_,0);
		_error[1] = rot_jerk;
	}
	void setConfig(const ConfigStruct& config)
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

class EdgeAccelerationStart : public g2o::BaseMultiEdge<2, double>
{
public:
	EdgeAccelerationStart()
	{
		this->resize(3);
	}
	
	virtual ~EdgeAccelerationStart()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}
	void computeError(void)
	{
		const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[2]);

		// VELOCITY & ACCELERATION
		const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
	
		double dist1 = diff1.norm();
		const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
		
		if (angle_diff1 != 0)
		{
			const double radius =  dist1/(2*sin(angle_diff1/2));
			dist1 = fabs( angle_diff1 * radius ); // actual arg length!
		}
		
		double vel1 = dist1 / dt1->dt();
		
		// consider directions
		vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
		
		const double acc_lin  = (vel1 - config_.max_start_vel_) / dt1->dt();

		_error[0] = penaltyBoundToInterval(acc_lin,config_.max_start_acc_,0);
// 		_error[0] = acc_jerk;
		
		// ANGULAR ACCELERATION
		const double omega1 = angle_diff1 / dt1->dt();
		const double acc_rot  = (omega1 - config_.max_start_omega_vel_)*2 /  dt1->dt();
		
		_error[1] = penaltyBoundToInterval(acc_rot,config_.max_start_omega_acc_,0);
// 		_error[1] = rot_jerk;
	}
	void setConfig(const ConfigStruct& config)
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

class EdgeAccelerationEnd : public g2o::BaseMultiEdge<2, double>
{
public:
	EdgeAccelerationEnd()
	{
		this->resize(3);
	}
	
	virtual ~EdgeAccelerationEnd()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}
	void computeError(void)
	{
		const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
		const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
		const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[2]);

		// VELOCITY & ACCELERATION
		const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
	
		double dist1 = diff1.norm();
		const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
		
		if (angle_diff1 != 0)
		{
			const double radius =  dist1/(2*sin(angle_diff1/2));
			dist1 = fabs( angle_diff1 * radius ); // actual arg length!
		}
		
		double vel1 = dist1 / dt1->dt();
		
		// consider directions
		vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
		
		const double acc_lin  = -(vel1 - config_.max_end_vel_) / dt1->dt();

		_error[0] = penaltyBoundToInterval(acc_lin,config_.max_end_acc_,0);
// 		_error[0] = acc_jerk;
		
		// ANGULAR ACCELERATION
		const double omega1 = angle_diff1 / dt1->dt();
		const double acc_rot  = -(omega1 - config_.max_end_omega_vel_) /  dt1->dt();
		
		_error[1] = penaltyBoundToInterval(acc_rot,config_.max_end_omega_acc_,0);
// 		_error[1] = rot_jerk;
	}
	void setConfig(const ConfigStruct& config)
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