#ifndef __WAYPOINT_HANDLE_H__
#define __WAYPOINT_HANDLE_H__

#include <fstream>
#include <iostream>
#include <string>
#include <hector_nav.h>
#include <vector>
#include "waypoint_type.h"

namespace hector_nav
{
	class waypoint_handle
	{
	public:
		waypoint_handle(std::string filepath)
		{
			//读取waypoint文件获取目标点以及悬停时间
			waypoint_file.open(filepath.c_str());
			if(!waypoint_file.is_open())
			{
				ROS_INFO("Cannot open the waypoint file!");
			}
			else
			{
				ROS_INFO("Waypoint file open successfully!");
				waypoint_type waypoint_;
				while(!waypoint_file.eof())
				{					
					waypoint_file >> waypoint_.waypoint.x >> waypoint_.waypoint.y 
								  >> waypoint_.waypoint.z >> waypoint_.stay_time;
					waypoint_vector.push_back(waypoint_);
				}
				waypoint_vector.pop_back();
				waypoint_file.close();
				
				//用于测试
				/*ROS_INFO("The number of the waypoints are: %d", (int)waypoint_vector.size());
				std::vector<waypoint_type>::iterator waypointpoint;
				
				for(waypointpoint = waypoint_vector.begin(); waypointpoint != waypoint_vector.end(); ++waypointpoint)
				{
					ROS_INFO("The stay time is %2.3f", waypointpoint->stay_time);
				}*/
			}			
		}
		virtual ~waypoint_handle()
		{}
		
		//返回当前轨迹点数目
		unsigned int get_waypoint_num(void)
		{
			return waypoint_vector.size();
		}
		
		//返回下一个目标点
		void get_nexttarget(waypoint_type &target)
		{
			if(waypoint_vector.size() != 0)
			{
				std::vector<waypoint_type>::iterator waypointpoint = waypoint_vector.begin(); 
				target.waypoint.x = waypointpoint->waypoint.x;
				target.waypoint.y = waypointpoint->waypoint.y;
				target.waypoint.z = waypointpoint->waypoint.z;
				target.stay_time  = waypointpoint->stay_time;
				waypoint_vector.erase(waypointpoint);
			}
		}
		void dynamicLoad(void)
		{
			if(waypoint_vector.size() == 0)
			{
				waypoint_type newPoint;
				std::string newline;
				std::cout << "Please input the new waypoint >" << std::endl;
				while(getline(std::cin, newline))
				{
					std::stringstream stream(newline);
					stream >> newPoint.waypoint.x >> newPoint.waypoint.y >> newPoint.waypoint.z;
					newPoint.stay_time = 1;
					waypoint_vector.push_back(newPoint);
					return;
				}
			}
		}
	private:
		std::ifstream              waypoint_file;
		//waypoint_type              current_waypoint;
		std::vector<waypoint_type> waypoint_vector;
	};
}

#endif