#include "octomap_construct.h"
#include <limits>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

// static inline double getYaw(const tf::Quaternion& bt_q){
//   tfScalar useless_pitch, useless_roll, yaw;
//   tf::Matrix3x3(bt_q).getRPY( useless_roll, useless_pitch,yaw);
//   return yaw;
// }

void OctomapConstruct::ConstructCloud(const sensor_msgs::CameraInfo& camera_info, const sensor_msgs::Image& depth_image, pcl::PointCloud< pcl::PointXYZ >& cloud)
{
	sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
	cloud_msg->header = depth_image.header;
	cloud_msg->height = depth_image.height;
	cloud_msg->width  = depth_image.width;
	cloud_msg->is_dense = false;
	cloud_msg->is_bigendian = false;

	sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
	pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(camera_info);
	//depth_image_proc::convert<float>(depthImage, cloud_msg, model,5.0);
	// Use correct principal point from calibration
	float center_x = model.cx();
	float center_y = model.cy();

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = depth_image_proc::DepthTraits<float>::toMeters( float(1) );
	float constant_x = unit_scaling / model.fx();
	float constant_y = unit_scaling / model.fy();
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	const float* depth_row = reinterpret_cast<const float*>(&depth_image.data[0]);
	int row_step = depth_image.step / sizeof(float);
	int cloud_cout = 0;
	for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
	{
		for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
		{
			float depth = depth_row[u];
			++cloud_cout;
			// Missing points denoted by NaNs
			if (!depth_image_proc::DepthTraits<float>::valid(depth))
			{
// 				*iter_x = *iter_y = *iter_z = bad_point;
				depth = depth_image_proc::DepthTraits<float>::fromMeters(5.0);
// 				continue;
			}

			// Fill in XYZ
			*iter_x = (u - center_x) * depth * constant_x;
			*iter_y = (v - center_y) * depth * constant_y;
			*iter_z = depth_image_proc::DepthTraits<float>::toMeters(depth);
		}
	}
	pcl::fromROSMsg(*cloud_msg,cloud);
}

void OctomapConstruct::ConstructMap(const pcl::PointCloud< pcl::PointXYZ >& cloud, const tf::StampedTransform& sensorToWorldTf, octomap::OcTree& map)
{
	int pc_count = 0;
	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
	pcl::PointCloud<pcl::PointXYZ> cloud_out;
	pcl::transformPointCloud(cloud,cloud_out,sensorToWorld);
	octomap::Pointcloud octomap_cloud;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud_out.begin(); it != cloud_out.end(); it++)
	{
		//用于进行it->z有效性判断
		if(it->z == it->z)
		{
			octomap::point3d point(it->x, it->y, it->z);
			if(pc_count%4==0)
				octomap_cloud.push_back(point);
			++pc_count;
		}
	}
	octomap::point3d sensorOrigin(sensorToWorldTf.getOrigin().x(),sensorToWorldTf.getOrigin().y(),sensorToWorldTf.getOrigin().z());
	insertScan(sensorToWorldTf.getOrigin(),cloud_out,map);

// 	map.writeBinary("/home/leon/source_code/gazebo_simulate/src/octomap_test/test.bt");
// 	std::cout << "The interval time of insertScan is: " << interval << std::endl;
}

void OctomapConstruct::insertScan(const tf::Point& origin, const pcl::PointCloud<pcl::PointXYZ>& cloud, 
								  octomap::OcTree& map)
{
	double max_range = 5;
	double minX, minY, minZ;
	double maxX, maxY, maxZ;
	octomap::OcTreeKey BBXMax, BBXMin;
	map.getMetricMin(minX, minY, minZ);
	map.getMetricMax(maxX, maxY, maxZ);

	BBXMin[0] = map.coordToKey(minX);
	BBXMin[1] = map.coordToKey(minY);
	BBXMin[2] = map.coordToKey(minZ);

	BBXMax[0] = map.coordToKey(maxX);
	BBXMax[1] = map.coordToKey(maxY);
	BBXMax[2] = map.coordToKey(maxZ);
	
	octomap::point3d sensor_origin(origin.x(),origin.y(),origin.z());
	if (!map.coordToKeyChecked(sensor_origin, BBXMin)
		|| !map.coordToKeyChecked(sensor_origin, BBXMax))
	{
		ROS_ERROR_STREAM("Could not generate Key for origin "<<sensor_origin);
	}
	//
	octomap::KeySet free_cells, occupied_cells;
	octomap::KeyRay key_ray;
	int pc_count = 0;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud.begin(); it != cloud.end(); it++)
	{
		//用于保证*it非nan
		if(it->z == it->z)
		{
			++pc_count;
			if(pc_count%10 == 0)
			{
				octomap::point3d point(it->x, it->y, it->z);
				if((point-sensor_origin).norm() > max_range)
				{
					double resolution = map.getResolution();
					point = sensor_origin + (point-sensor_origin).normalized()*max_range;
					if (map.computeRayKeys(sensor_origin, point, key_ray))
					{
						free_cells.insert(key_ray.begin(), key_ray.end());
					}
				}
				else
				{
					if (map.computeRayKeys(sensor_origin, point, key_ray))
					{
						free_cells.insert(key_ray.begin(), key_ray.end());
					}
					octomap::OcTreeKey key;
					if (map.coordToKeyChecked(point, key))
					{
						occupied_cells.insert(key);
					}
				}
			}
		}
	}
// 	std::cout << "The cloud count is : " << pc_count << std::endl;
	  // mark free cells only if not seen occupied in this cloud
	for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; it++)
	{
		if (occupied_cells.find(*it) == occupied_cells.end())
		{
			map.updateNode(*it, false);
		}
	}

	// now mark all occupied cells:
	for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) 
	{
		map.updateNode(*it, true);
// 		octomap::point3d point = map.keyToCoord(*it);
// 		if(point.z() > 0.4)
// 		{
// 			std::cout << "The obstacle coo is: " << point << std::endl;
// 		}
		
	}
	map.prune();
}

void OctomapConstruct::ConstructGridMap(const octomap::OcTree& _octomap, const tf::StampedTransform& baseToWorldTf, grid_map::GridMap& grid_map)
{
	double detect_distance = 5.0f;
	double detect_angle    = 70;
	double detect_upbound  = 0.2f;
	double detect_downbound= -0.3f;
	double detect_width    = detect_distance*tan(detect_angle*0.0087266461f);
	grid_map::Position origin(baseToWorldTf.getOrigin().x(),baseToWorldTf.getOrigin().y());
	grid_map.move(origin);
	grid_map["probability"].setConstant(NAN);
	grid_map::Polygon polygon;
	polygon.setFrameId(grid_map.getFrameId());
	double yaw = getYaw(baseToWorldTf.getRotation());
//----------------------普通转换方式------------------------------
	Eigen::Matrix3d transpose;
	transpose << cos(yaw), -sin(yaw), origin(0),
				 sin(yaw),  cos(yaw), origin(1),
				 0,         0,        1;
//--------------------------------------------------------------
// 		Eigen::Isometry2d transform2d(Eigen::Translation2d(flight_state_.PosX,flight_state_.PosY)*
// 									  Eigen::Rotation2D<double>(flight_state_.Yaw/180.0f*3.1415926f));

	Eigen::Vector3d leftFar;
	Eigen::Vector3d rightFar;
	Eigen::Vector3d staPoint;
	
	leftFar  << detect_distance,   -detect_width,  1;
	rightFar << detect_distance,    detect_width,  1;
	staPoint << 0,                  0,             1;
//-----------------------普通转换方式-----------------------------
	leftFar   = transpose*leftFar;
	rightFar  = transpose*rightFar;
	staPoint  = transpose*staPoint;
//--------------------------------------------------------------
// 		leftNear  = transform2d*leftNear;
// 		rightNear = transform2d*rightNear;
// 		leftFar   = transform2d*leftFar;
// 		rightFar  = transform2d*rightFar;
	
	polygon.addVertex(grid_map::Position(leftFar(0),leftFar(1)));
	polygon.addVertex(grid_map::Position(rightFar(0),rightFar(1)));
	polygon.addVertex(grid_map::Position(staPoint(0),staPoint(1)));

	for (grid_map::/*PolygonIterator*/GridMapIterator iterator(grid_map/*, polygon*/);
		!iterator.isPastEnd(); ++iterator) 
	{
		auto& grid_value = grid_map.at("probability",*iterator);
		grid_map::Position position2d;
		grid_map.getPosition(*iterator, position2d);
		
		double flight_height = baseToWorldTf.getOrigin().z();
		for(double detect_height = flight_height+detect_upbound; 
			detect_height > flight_height+detect_downbound; 
			detect_height -= _octomap.getResolution())
		{
			octomap::OcTreeNode* node = _octomap.search(position2d(0),position2d(1),detect_height);
			if(node)
			{
				if(_octomap.isNodeOccupied(*node))
				{
					grid_value = 10;
				}
				else
				{	
					grid_value = 0;
				}
			}
		}
	}
// 	grid_map::Index origin_index;
// 	grid_map.getIndex(origin, origin_index);
// 	grid_map.at("probability",origin_index) = 10;
}
