#include "trajectory_generate.h"

namespace hector_nav
{
	void trajectory_generate::updateWaypointInfo(const waypointInfo& initialPoint, const waypointInfo& finalPoint, 
												 const double &flightTime)
	{
		initialPoint_ = initialPoint;
		finalPoint_   = finalPoint;
		flightTime_   = flightTime;
		
		Eigen::Vector3d detaXAxis;
		Eigen::Vector3d detaYAxis;
		Eigen::Vector3d detaZAxis;
		
		detaXAxis(0) = finalPoint_.xAxis.a - initialPoint_.xAxis.a;
		detaXAxis(1) = finalPoint_.xAxis.v - initialPoint_.xAxis.v - initialPoint_.xAxis.a*flightTime_;
		detaXAxis(2) = finalPoint_.xAxis.p - initialPoint_.xAxis.p - initialPoint_.xAxis.v*flightTime_ 
		               - 0.5*initialPoint_.xAxis.a*flightTime_*flightTime_;
		
		detaYAxis(0) = finalPoint_.yAxis.a - initialPoint_.yAxis.a;
		detaYAxis(1) = finalPoint_.yAxis.v - initialPoint_.yAxis.v - initialPoint_.yAxis.a*flightTime_;
		detaYAxis(2) = finalPoint_.yAxis.p - initialPoint_.yAxis.p - initialPoint_.yAxis.v*flightTime_ 
		               - 0.5*initialPoint_.yAxis.a*flightTime_*flightTime_;
		
		detaZAxis(0) = finalPoint_.zAxis.a - initialPoint_.zAxis.a;
		detaZAxis(1) = finalPoint_.zAxis.v - initialPoint_.zAxis.v - initialPoint_.zAxis.a*flightTime_;
		detaZAxis(2) = finalPoint_.zAxis.p - initialPoint_.zAxis.p - initialPoint_.zAxis.v*flightTime_ 
		               - 0.5*initialPoint_.zAxis.a*flightTime_*flightTime_;
					   
		Eigen::Matrix3d converter;
		converter << 60*pow(flightTime_,2), -360*flightTime_,         720,
		            -24*pow(flightTime_,3),  168*pow(flightTime_,2), -360*flightTime_,
		              3*pow(flightTime_,4),  -24*pow(flightTime_,3),   60*pow(flightTime_,2);
	
	    Eigen::Vector3d detaAxis;
		detaAxis = 1/pow(flightTime_,5) * converter * detaXAxis;
		xAxis.alpha = detaAxis(0);
		xAxis.beta  = detaAxis(1);
		xAxis.gamma = detaAxis(2);

		detaAxis = 1/pow(flightTime_,5) * converter * detaYAxis;
		yAxis.alpha = detaAxis(0);
		yAxis.beta  = detaAxis(1);
		yAxis.gamma = detaAxis(2);

		detaAxis = 1/pow(flightTime_,5) * converter * detaZAxis;
		zAxis.alpha = detaAxis(0);
		zAxis.beta  = detaAxis(1);
		zAxis.gamma = detaAxis(2);		
	}

	geometryLinear_t trajectory_generate::getCurrentSingleOut(const geometryLinear_t& initial, const trajParam_t &param,
															  const double& time)
	{
		geometryLinear_t output;
		output.p = param.alpha/120*pow(time,5) + param.beta/24*pow(time,4) + param.gamma/6*pow(time,3)
		         + initial.a/2*pow(time,2) + initial.v*time + initial.p;
		output.v = param.alpha/ 24*pow(time,4) + param.beta/ 6*pow(time,3) + param.gamma/2*pow(time,2)
		         + initial.a*time + initial.v;
		output.a = param.alpha/  6*pow(time,3) + param.beta/ 2*pow(time,2) + param.gamma*time
		         + initial.a;
		return output;
	}
	
	waypointInfo trajectory_generate::getCurrentOut(const double& time)
	{
		waypointInfo output;
		output.xAxis = getCurrentSingleOut(initialPoint_.xAxis, xAxis, time);
		output.yAxis = getCurrentSingleOut(initialPoint_.yAxis, yAxis, time);
		output.zAxis = getCurrentSingleOut(initialPoint_.zAxis, zAxis, time);
		return output;
	}

}