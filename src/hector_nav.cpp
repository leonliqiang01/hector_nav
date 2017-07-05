#include "hector_nav.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"
// #include "hector_nav2/FeedbackMsg.h"
#include "hector_nav2/TrajectoryMsg.h"
#include <chrono>

std::string filepath        = "";
const double control_fre    = 20;                    //主要代码执行频率
const double control_period = 1.0f/control_fre; 
const double hover_accuracy = 0.2f;                  //悬停精度
const double turn_accuracy  = (1.0f*3.1415926f)/180.0f; //方向精度

double getYaw(const geometry_msgs::Pose &pose)
{
	const geometry_msgs::Quaternion::_w_type& w = pose.orientation.w;
	const geometry_msgs::Quaternion::_w_type& x = pose.orientation.x;
	const geometry_msgs::Quaternion::_w_type& y = pose.orientation.y;
	const geometry_msgs::Quaternion::_w_type& z = pose.orientation.z;
	return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

namespace hector_nav 
{
	hector_nav_handle::hector_nav_handle():control_handle("~"),state_sub(new message_filters::Subscriber<nav_msgs::Odometry>(control_handle,"/ground_truth/state",1)),
	                   pointCloud_sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(control_handle,"/camera/depth/points",1)),
	                   sync(new  message_filters::Synchronizer<navSyncPolicy>(navSyncPolicy(10),*state_sub,*pointCloud_sub)),
	                   waypoint_handle_(filepath),hover_time(0),cloud_ID_("/camera/depth/points"),world_frame_ID_("world"),depth_image_ID_("/camera/depth/image_raw"),
				       fx_(319.9988f),fy_(319.9988f),cx_(320.5f),cy_(240.5f),octomap_(0.5),octomap_construct_(fx_,fy_,cx_,cy_),
				       grid_map_({"probability"}),teb_obstacle_vec_(std::make_shared<std::vector<ObstacleStruct>>()),
				       teb_planner_(teb_config_,teb_obstacle_vec_)
	{
		//进行同步设置
		sync->registerCallback(boost::bind(&hector_nav_handle::messageCallback,this,_1,_2));
		
		vel_pub = control_handle.advertise<geometry_msgs::Twist>("/cmd_vel",10);
		reConfig.setCallback(boost::bind(&hector_nav_handle::reConfigCallback,this,_1,_2));
		//读取控制器参数
		control_handle.param<double>("/controller/position/xy/k_p", pid_xy.k_p, 1.5);
		control_handle.param<double>("/controller/position/xy/k_i", pid_xy.k_i, 0.0);
		control_handle.param<double>("/controller/position/xy/k_d", pid_xy.k_d, 0.0);
		control_handle.param<double>("/controller/position/xy/limit_output", pid_xy.limit, 2.0);
			
		control_handle.param<double>("/controller/position/z/k_p", pid_z.k_p, 1.5);
		control_handle.param<double>("/controller/position/z/k_i", pid_z.k_i, 0.0);
		control_handle.param<double>("/controller/position/z/k_d", pid_z.k_d, 0.0);
		control_handle.param<double>("/controller/position/z/limit_output", pid_z.limit, 2.0);
		
		control_handle.param<double>("/controller/position/yaw/k_p", pid_yaw.k_p, 1.2);
		control_handle.param<double>("/controller/position/yaw/k_i", pid_yaw.k_i, 0.0);
		control_handle.param<double>("/controller/position/yaw/k_d", pid_yaw.k_d, 0.8);
		control_handle.param<double>("/controller/position/yaw/limit_output", pid_yaw.limit, 2.0);
		
		position_control_.set_pidxy_parameters(pid_xy);
		position_control_.set_pidz_parameters(pid_z);
		position_control_.set_pidyaw_parameters(pid_yaw);
		
		depth_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (control_handle, depth_image_ID_, 5);
		tf_depth_image_sub_ = new tf::MessageFilter<sensor_msgs::Image> (*depth_image_sub_, tf_listener_, world_frame_ID_, 5);
		tf_depth_image_sub_->registerCallback(boost::bind(&hector_nav_handle::depthImageCallback, this, _1));
		
		point_cloud_pub_ = control_handle.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 3);
		depth_image_pub_ = control_handle.advertise<sensor_msgs::Image>("octomap_camera_depth", 3);
		camera_info_sub_ = control_handle.subscribe("/camera/depth/camera_info",5,&hector_nav_handle::cameraInfoCallback, this);
		marker_pub_   = control_handle.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1);
		grid_map_pub_ = control_handle.advertise<grid_map_msgs::GridMap>("grid_map",1,true);
		
		local_plan_pub_ = control_handle.advertise<nav_msgs::Path>("local_plan",1);
		teb_poses_pub_  = control_handle.advertise<geometry_msgs::PoseArray>("teb_poses", 100);
		feedback_pub_   = control_handle.advertise<hector_nav2::TrajectoryMsg>("teb_feedback", 10);  
			
		pcl_point_cloud_.clear();
		
		grid_map_.setFrameId(world_frame_ID_);
		grid_map_.setGeometry(grid_map::Length(30, 30), 0.5, grid_map::Length(0,0));
		
		ROS_INFO("Controller parameter is loaded completely!");
		//飞行状态初始化
		flight_state = STOP;
	}
	void hector_nav_handle::reConfigCallback(hector_nav::param_reconfigConfig &config, uint32_t level)
	{
	    pid_xy.k_p = config.xy_kp;
		pid_xy.k_i = config.xy_ki;
		pid_xy.k_d = config.xy_kd;
		pid_xy.limit = config.xy_limit;
		
		pid_z.k_p = config.z_kp;
		pid_z.k_i = config.z_ki;
		pid_z.k_d = config.z_kd;
		pid_z.limit = config.z_limit;
		
		pid_yaw.k_p = config.yaw_kp;
		pid_yaw.k_i = config.yaw_ki;
		pid_yaw.k_d = config.yaw_kd;
		pid_yaw.limit = config.yaw_limit;
		
		cofFlightTime = config.cof_flighttime;
		cofAcce       = config.cof_acce;
		planVel       = config.plan_vel;
	}

	//导航规划主函数
	void hector_nav_handle::update_handle(void)
	{	
		//具体的导航规划
		double yaw = 0;
		if(flight_state == READY)
		{
			if(waypoint_handle_.get_waypoint_num() != 0)
			{
				waypoint_handle_.get_nexttarget(current_waypoint);
				flight_state = CLIMBING;
				position_control_.command_yaw = getYaw(current_state.pose.pose);
				ROS_INFO("Ready to run!");
			}
		}
		if(flight_state == CLIMBING)
		{
			current_command.pose.pose.position.z = current_waypoint.waypoint.z;	
			ROS_INFO("Climbing!");
			if((current_command.pose.pose.position.z == 0.0f) && (current_state.twist.twist.linear.z <= 0.1f))
			{
				ROS_INFO("Landing!");
				flight_state = STOP;
				current_command.pose.pose.position.z = current_state.pose.pose.position.z;
			}
			if(fabs(current_command.pose.pose.position.z - current_state.pose.pose.position.z) 
				   < hover_accuracy)
			{		
				double distance = sqrt(pow(current_waypoint.waypoint.x - current_state.pose.pose.position.x,2)+
					                   pow(current_waypoint.waypoint.y - current_state.pose.pose.position.y,2));
				//flight_state = RUNNING;
				//在目标点与当前点距离超过0.2米时才进行旋转
				if(distance > 0.2)
				{
					flight_state = TURNING;		
					position_control_.yaw_control_flag = true;
					yaw = atan2(current_waypoint.waypoint.y - current_state.pose.pose.position.y,
								current_waypoint.waypoint.x - current_state.pose.pose.position.x);
					position_control_.command_yaw = yaw;
					ROS_INFO("Current yaw is: %2.5f", getYaw(current_state.pose.pose));
					ROS_INFO("Current command yaw is: %2.5f", position_control_.command_yaw);
				}
				else
				{
					flight_state = RUNNING;
					//planner_port_.setRunMode(Guidance::GLOBAL);
// 					initialTrajectory();
					OAMode = NO_OBSTACLE;
				}
			}
		}
		if(flight_state == TURNING)
		{
			ROS_INFO("Turning!");
			//ROS_INFO("The target yaw is: %2.5f", yaw);
			if(fabs(getYaw(current_state.pose.pose) - position_control_.command_yaw) < turn_accuracy)
			{
				ROS_INFO("Start to run!");
				ROS_INFO("Current yaw is: %2.5f", fabs(getYaw(current_state.pose.pose) - position_control_.command_yaw));
				ROS_INFO("Current command yaw is: %2.5f", turn_accuracy);
// 				initialTrajectory();
				flight_state = RUNNING;
				//planner_port_.setRunMode(Guidance::GLOBAL);
				position_control_.yaw_control_flag = false;
			}
		}
		if(flight_state == RUNNING)
		{
			PlanStruct current_plan;
			current_plan.current_pose = current_state.pose.pose;
			current_plan.current_vel = current_state.twist.twist;
			current_plan.goal_pose.orientation = tf::createQuaternionMsgFromYaw(0);
			current_plan.goal_pose.position.x = current_waypoint.waypoint.x;
			current_plan.goal_pose.position.y = current_waypoint.waypoint.y;
			current_plan.goal_vel = geometry_msgs::Twist();
			
			double distance = sqrt(pow(current_plan.goal_pose.position.x - current_state.pose.pose.position.x,2)+
					               pow(current_plan.goal_pose.position.y - current_state.pose.pose.position.y,2));
			if(distance < 0.2)
			{
				flight_state = STOP;
			}
			else
			{
				std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
				teb_planner_.plan(current_plan, current_command);
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
				std::cout << "solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
			}
		}
		if(flight_state == STOP)
		{
			if(hover_time * control_period < current_waypoint.stay_time)
			{
				++hover_time;
				//ROS_INFO("Hovering: %d", hover_time);
			}
			else if(waypoint_handle_.get_waypoint_num() != 0)
			{
				flight_state = READY;
				hover_time   = 0;
				ROS_INFO("Load another waypoint!");
			}
			if(waypoint_handle_.get_waypoint_num() == 0)
			{
				position_control_.stop(vel_output);
				vel_pub.publish(vel_output);
				//waypoint_handle_.dynamicLoad();
				//ROS_INFO("No waypoint to load!");
			}
		}
		if(1/*flight_state != RUNNING*/)
		{
			position_control_.update_controller(current_command, current_state, vel_output);
// 			vel_output.linear.y = 0;
		}
		else
		{
			vel_output = current_command.twist.twist;
		}
		vel_pub.publish(vel_output);
		//ROS_INFO("%2.3f, %2.3f, %2.3f", vel_output.linear.x, vel_output.linear.y, vel_output.linear.z);
	}

	void hector_nav_handle::depthImageCallback(const sensor_msgs::ImageConstPtr& depthImage)
	{
		pcl_point_cloud_.clear();
		tf::StampedTransform sensorToWorldTf;
		try {
			tf_listener_.lookupTransform(world_frame_ID_, depthImage->header.frame_id, depthImage->header.stamp, sensorToWorldTf);
		} catch(tf::TransformException& ex){
			ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
			return;
		}
		tf::StampedTransform baseToWorldTf;
		try{
			tf_listener_.lookupTransform(world_frame_ID_, "base_link", depthImage->header.stamp, baseToWorldTf);
		} catch(tf::TransformException& ex){
			ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quittiong callback"); 
			return;
		}
		
// 		ros::WallTime begin = ros::WallTime::now();
		octomap_construct_.ConstructCloud(camera_info_,*depthImage,pcl_point_cloud_);
		octomap_construct_.ConstructMap(pcl_point_cloud_, sensorToWorldTf, octomap_);
// 		ros::WallTime end = ros::WallTime::now();
		octomap_construct_.ConstructGridMap(octomap_, baseToWorldTf, grid_map_);

// 		double intervel = (end-begin).toNSec();
// 		std::cout << "The intervel is: " << intervel << std::endl;

// 		std::cout << "The size of the cloud is: " << pcl_point_cloud_.size() << std::endl;
		sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(pcl_point_cloud_, *cloud_msg);
		point_cloud_pub_.publish(*cloud_msg);
		grid_map_.setTimestamp(depthImage->header.stamp.toNSec());
// 		std::cout << "The cloud size is: " << pcl_point_cloud_.size() << std::endl;
		grid_map_msgs::GridMap message;
		grid_map::GridMapRosConverter::toMessage(grid_map_, message);
		grid_map_pub_.publish(message);

		update_handle();
		SearchObstacle();
		publishLocalPlanAndPoses();
		PublishMarkerArray(depthImage->header.stamp);
		publishFeedBack();
	}

	void hector_nav_handle::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo)
	{
		camera_info_ = *cameraInfo;
	}

	void hector_nav_handle::SearchObstacle(void)
	{
		if(current_state.pose.pose.orientation.w == 0
		 &&current_state.pose.pose.orientation.x == 0
		 &&current_state.pose.pose.orientation.y == 0
		 &&current_state.pose.pose.orientation.z == 0)
			return;
		double yaw = tf::getYaw(current_state.pose.pose.orientation);
		double search_width_ = 3;
		double search_distance_ = 6;
		grid_map::Polygon polygon;
//----------------------普通转换方式------------------------------
		Eigen::Matrix3d transpose;
		transpose << cos(yaw), -sin(yaw), current_state.pose.pose.position.x,
					 sin(yaw),  cos(yaw), current_state.pose.pose.position.y,
					 0,         0,        1;
//--------------------------------------------------------------
// 		Eigen::Isometry2d transform2d(Eigen::Translation2d(flight_state_.PosX,flight_state_.PosY)*
// 									  Eigen::Rotation2D<double>(flight_state_.Yaw/180.0f*3.1415926f));
		Eigen::Vector3d leftNear;
		Eigen::Vector3d rightNear;
		Eigen::Vector3d leftFar;
		Eigen::Vector3d rightFar;
		
		leftNear  << 0,                 -search_width_, 1;
		rightNear << 0,           		 search_width_, 1;
		leftFar   << search_distance_,  -search_width_, 1;
		rightFar  << search_distance_,   search_width_, 1;
//-----------------------普通转换方式-----------------------------
		leftNear  = transpose*leftNear;
		rightNear = transpose*rightNear;
		leftFar   = transpose*leftFar;
		rightFar  = transpose*rightFar;
//--------------------------------------------------------------
// 		leftNear  = transform2d*leftNear;
// 		rightNear = transform2d*rightNear;
// 		leftFar   = transform2d*leftFar;
// 		rightFar  = transform2d*rightFar;
		
		polygon.addVertex(grid_map::Position(leftNear(0),leftNear(1)));
		polygon.addVertex(grid_map::Position(rightNear(0),rightNear(1)));
		polygon.addVertex(grid_map::Position(rightFar(0),rightFar(1)));
		polygon.addVertex(grid_map::Position(leftFar(0),leftFar(1)));
		teb_obstacle_vec_->clear();
		for (grid_map::PolygonIterator iterator(grid_map_, polygon);
			!iterator.isPastEnd(); ++iterator)
		{
			auto& grid_value = grid_map_.at("probability", *iterator);
			if(grid_value == grid_value)
			{
				//识别为障碍物
				if(grid_value > 0)
				{
					grid_map::Position currentPoint;
					grid_map_.getPosition(*iterator, currentPoint);
					teb_obstacle_vec_->push_back(ObstacleStruct(currentPoint));
				}
			}
		}
	}

	void hector_nav_handle::PublishMarkerArray(const ros::Time& ros_time)
	{
		visualization_msgs::MarkerArray freeNodesVis;
		freeNodesVis.markers.resize(octomap_.getTreeDepth()+1);
		visualization_msgs::MarkerArray occupiedNodesVis;
		occupiedNodesVis.markers.resize(octomap_.getTreeDepth()+1);
		
		for (octomap::OcTree::iterator it = octomap_.begin(octomap_.getTreeDepth()),
		end = octomap_.end(); it != end; ++it)
		{
			if(octomap_.isNodeOccupied(*it))
			{
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();
				unsigned idx = it.getDepth();
				assert(idx < occupiedNodesVis.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				occupiedNodesVis.markers[idx].points.push_back(cubeCenter);	
			}
			else
			{
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();
				unsigned idx = it.getDepth();
				assert(idx < freeNodesVis.markers.size());
				
				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;
				freeNodesVis.markers[idx].points.push_back(cubeCenter);
			}
		}
		std_msgs::ColorRGBA m_color;
		m_color.r = 0.0;
		m_color.g = 0.0;
		m_color.b = 1.0;
		m_color.a = 1.0;
		for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
		{
			double size = octomap_.getNodeSize(i);

			occupiedNodesVis.markers[i].header.frame_id = world_frame_ID_;
			occupiedNodesVis.markers[i].header.stamp = ros_time;
			occupiedNodesVis.markers[i].ns = "map";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;
			occupiedNodesVis.markers[i].color = m_color;

			if (occupiedNodesVis.markers[i].points.size() > 0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		marker_pub_.publish(occupiedNodesVis);
	}

	void hector_nav_handle::publishLocalPlanAndPoses(void)
	{
		nav_msgs::Path teb_path;
		teb_path.header.frame_id = world_frame_ID_;
		teb_path.header.stamp = ros::Time::now();
		
		geometry_msgs::PoseArray teb_poses;
		teb_poses.header.frame_id = teb_path.header.frame_id;
		teb_poses.header.stamp = teb_path.header.stamp;
		
		for (int i=0; i < teb_planner_.SizePoseVec(); i++)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = teb_path.header.frame_id;
			pose.header.stamp = teb_path.header.stamp;
			pose.pose.position.x = teb_planner_.PoseVertex(i)->x();
			pose.pose.position.y = teb_planner_.PoseVertex(i)->y();
			pose.pose.position.z = 0;
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb_planner_.PoseVertex(i)->theta());
			teb_path.poses.push_back(pose);
			teb_poses.poses.push_back(pose.pose);
		}
		local_plan_pub_.publish(teb_path);
		teb_poses_pub_.publish(teb_poses);
	}
	void hector_nav_handle::publishFeedBack(void)
	{
		hector_nav2::TrajectoryMsg _trajectory;
		_trajectory.header.frame_id = world_frame_ID_;
		_trajectory.header.stamp    = ros::Time::now();

		teb_planner_.CalTrajectory(_trajectory);
		feedback_pub_.publish(_trajectory);
// 		_trajectory.header.frame_id = world_frame_ID_;
// 		_trajectory.header.stamp = ros::Time::now();
// 		for(int i=0; i < teb_planner_.SizePoseVec(); i++)
// 		{
// 			hector_nav2::TrajectoryPointMsg _trajectory_point;
// 			geometry_msgs::Pose pose;
// 			pose.position.x = teb_planner_.PoseVertex(i)->x();
// 			pose.position.y = teb_planner_.PoseVertex(i)->y();
// 			pose.position.z = 0;
// 			pose.orientation = tf::createQuaternionFromYaw(teb_planner_.PoseVertex(i)->theta());
// 			_trajectory_point.pose = pose;
// 			_trajectory.trajectory.push_back(_trajectory_point);
// 		}
	}
}

using namespace hector_nav;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hector_nav");
	if(argc > 1)
	{
		filepath = argv[1];
	}
	hector_nav_handle p_;
	ros::Duration(5).sleep();
	ros::Rate loop_rate(control_fre);
	
	ros::spin();
// 	while(ros::ok())
// 	{
// 		ros::spinOnce();
// 		p_.update_handle();
// 		loop_rate.sleep();
// 	}
	return 0;
}
