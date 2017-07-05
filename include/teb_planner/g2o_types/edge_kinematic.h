#ifndef __EDGE_KINEMATIC_H__
#define __EDGE_KINEMATIC_H__

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types/penalties.h"

#include "teb_config.h"

class EdgeKinematic : public g2o::BaseBinaryEdge<2,double,VertexPose,VertexPose>
{
public:
	EdgeKinematic()
	{
		this->resize(2);
	}
	virtual ~EdgeKinematic()
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
		
		Eigen::Vector2d deltaS = conf2->position() - conf1->position();

		// non holonomic constraint
		_error[0] = fabs( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );

		// positive-drive-direction constraint
		Eigen::Vector2d angle_vec ( cos(conf1->theta()), sin(conf1->theta()) );	   
		_error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0,0);
	}
	
	bool read(std::istream& is)
	{
		return true;
	}
	bool write(std::ostream& os) const
	{
		return os.good();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif