#ifndef __EDGE_OBSTACLE_H__
#define __EDGE_OBSTACLE_H__

#include <g2o/core/base_unary_edge.h>
#include <cmath>

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"
#include "g2o_types/penalties.h"

#include "teb_config.h"

class EdgeObstacle : public g2o::BaseUnaryEdge<1,ObstacleStruct,VertexPose>
{
public:
	EdgeObstacle()
	{
		this->resize(1);
	}
	virtual ~EdgeObstacle()
	{
		for(std::size_t i=0; i<_vertices.size(); ++i)
		{
			if(_vertices[i])
				_vertices[i]->edges().erase(this);
		}
	}
	void computeError(void)
	{
		const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
		Eigen::Vector2d obstacle_position = _measurement.getPosition();
		Eigen::Vector2d dist = (obstacle_position - bandpt->position());
		_error[0] = penaltyBoundFromBelow(dist.norm(), config_.min_obstacle_dist_, 0);
	}
	void setConfig(ConfigStruct& config)
	{
		config_ = config;
	}
	
	void setObstacle(const ObstacleStruct& obstacle)
	{
		_measurement = obstacle;
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