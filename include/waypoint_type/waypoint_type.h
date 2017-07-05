#ifndef __WAYPOINT_TYPE_H__
#define __WAYPOINT_TYPE_H__

#include "geometry_msgs/Point.h"

struct waypoint_type
{
	geometry_msgs::Point waypoint;
	double               stay_time;
};

#endif