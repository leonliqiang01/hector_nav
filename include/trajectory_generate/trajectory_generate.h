#ifndef __TRAJECTORY_GENERATE_H__
#define __TRAJECTORY_GENERATE_H__

#include <Eigen/Dense>
#include <iostream>

namespace hector_nav
{
	class geometryLinear_t
	{
	public:
		double p;
		double v;
		double a;
	};
	class trajParam_t
	{
	public:
		double alpha;
		double beta;
		double gamma;
	};
	class waypointInfo
	{
	public:
		geometryLinear_t xAxis;
		geometryLinear_t yAxis;
		geometryLinear_t zAxis;
	};
	class trajectory_generate
	{
	public:
		trajectory_generate(){}
		virtual ~trajectory_generate(){}
		void updateWaypointInfo(const waypointInfo &initialPoint, const waypointInfo &finalPoint, 
								const double &flightTime);
		waypointInfo getCurrentOut(const double &time);
		
	private:
		geometryLinear_t getCurrentSingleOut(const geometryLinear_t& initial, const trajParam_t &param,
											 const double& time);
		
		trajParam_t xAxis;
		trajParam_t yAxis;
		trajParam_t zAxis;
		
		waypointInfo initialPoint_;
		waypointInfo finalPoint_;
		double       flightTime_;
	};
}

#endif