#ifndef __DELTA_T_H__
#define __DELTA_T_H__

#include <g2o/core/base_unary_edge.h>
#include <cmath>

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"
#include "g2o_types/penalties.h"

#include "teb_config.h"

class EdgeTimeDiff : public g2o::BaseUnaryEdge<1,double,VertexTimeDiff>
{
public:
	EdgeTimeDiff()
	{
		this->resize(1);
	}
	virtual ~EdgeTimeDiff()
	{
		if(_vertices[0])
			_vertices[0]->edges().erase(this);
	}

	void computeError()
	{
		VertexTimeDiff* time_diff = static_cast<VertexTimeDiff*>(_vertices[0]);
		_error[0] = time_diff->estimate();
	}
	void linearizeOplus()
	{
		_jacobianOplusXi( 0 , 0 ) = 1;
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