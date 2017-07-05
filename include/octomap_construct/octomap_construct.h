#ifndef __OCTOMAP_CONSTRUCT_H__
#define __OCTOMAP_CONSTRUCT_H__

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

#include <octomap/OcTree.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_core/grid_map_core.hpp"

class OctomapConstruct
{
public:
	OctomapConstruct(float fx, float fy, float cx, float cy):fx_(fx),fy_(fy),cx_(cx),cy_(cy){}
	virtual ~OctomapConstruct(){}
	void ConstructCloud(const sensor_msgs::CameraInfo& camera_info, const sensor_msgs::Image& depth_image,
						pcl::PointCloud<pcl::PointXYZ>& cloud);
	void ConstructMap(const pcl::PointCloud<pcl::PointXYZ>& cloud, const tf::StampedTransform& sensorToWorldTf, 
					  octomap::OcTree& map);
	void ConstructGridMap(const octomap::OcTree& _octomap, const tf::StampedTransform& baseToWorldTf, 
						  grid_map::GridMap& grid_map);
	void insertScan(const tf::Point& origin, const pcl::PointCloud<pcl::PointXYZ>& cloud, 
								      octomap::OcTree& map);
protected:
	float fx_;
	float fy_;
	float cx_;
	float cy_;
};

#endif