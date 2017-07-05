#ifndef __HECTOR_NAV_H__
#define __HECTOR_NAV_H__

#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "position_control.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#include <message_filters/subscriber.h> 
#include "message_filters/connection.h"
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>  

#include "position_control.h"
#include "waypoint_type.h"
#include "waypoint_handle.h"
#include "sensor_msgs/PointCloud2.h"
// #include "planner_port.h"
#include "trajectory_generate.h"
// #include "planner_base.h"

#include <dynamic_reconfigure/server.h>
#include <hector_nav/param_reconfigConfig.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/OcTree.h>
#include "octomap_construct.h"
#include "teb_planner.h"

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace hector_nav
{
	class hector_nav_handle
	{
	public:
		//监听状态值回调函数
		void messageCallback(const nav_msgs::OdometryConstPtr& odometryMsg,const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
		{
			current_state = *odometryMsg;
			pointCloud    = *cloudMsg;
		}
		hector_nav_handle();
		virtual ~hector_nav_handle()
		{}
		void update_handle(void);
		void publishLocalPlanAndPoses(void);
		void publishFeedBack(void);
		
// 		void initialTrajectory(void);
// 		void initialOATrajectory(void);
		void reConfigCallback(hector_nav::param_reconfigConfig &config, uint32_t level);
		
// 		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
		void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_image);
		void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
		void SearchObstacle(void);
		void PublishMarkerArray(const ros::Time& ros_time);
			
	private:
		
		typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> navSyncPolicy;
		
		nav_msgs::Odometry       current_command;
		nav_msgs::Odometry       current_state;
		nav_msgs::Odometry       currentOACommand;
		sensor_msgs::PointCloud2 pointCloud;
		geometry_msgs::Twist     vel_output;
		ros::NodeHandle          control_handle;
		ros::Publisher           vel_pub;
		
		//订阅信息进行同步
		std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> >     state_sub;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > pointCloud_sub;
		std::shared_ptr<message_filters::Synchronizer<navSyncPolicy> > sync;
		
		dynamic_reconfigure::Server<hector_nav::param_reconfigConfig> reConfig;
		
// 		trajectory_generate     trajectory_;
		waypointInfo            currentOutput;
		//planner_port            planner_port_;
		position_control        position_control_;
		waypoint_handle         waypoint_handle_;
		waypoint_type           current_waypoint;
		pid_t                   pid_xy, pid_z, pid_yaw;	
		double                  currentFlightTime;
		int                     hover_time;
		
		double cofFlightTime;
		double cofAcce;
		double planVel;
		
		tf::TransformListener tf_listener_;
		
		message_filters::Subscriber<sensor_msgs::Image>* depth_image_sub_;
		tf::MessageFilter<sensor_msgs::Image>* tf_depth_image_sub_;
		
		ros::Subscriber camera_info_sub_;
		
		pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud_;

		ros::Publisher point_cloud_pub_;
		ros::Publisher depth_image_pub_;
		ros::Publisher marker_pub_;
		ros::Publisher local_plan_pub_;
		ros::Publisher teb_poses_pub_; 
		ros::Publisher feedback_pub_;
		std::string cloud_ID_;
		std::string world_frame_ID_;	
		std::string depth_image_ID_;
		
		sensor_msgs::CameraInfo camera_info_;
		
		float fx_;
		float fy_;
		float cx_;
		float cy_;
		octomap::OcTree octomap_;
		OctomapConstruct octomap_construct_;
		
		grid_map::GridMap grid_map_;
		ros::Publisher    grid_map_pub_;
		ConfigStruct      teb_config_;
		ObstacleVecPtr    teb_obstacle_vec_;
		TebPlannerTest    teb_planner_;

		
		enum
		{
			READY = 0,
			CLIMBING,
			TURNING,
			RUNNING,
			STOP
		} flight_state;
		enum
		{
			NO_OBSTACLE = 0,
			DETECT_OBSTACLE
		}OAMode;
		
	};
}

#endif