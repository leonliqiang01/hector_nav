#include "teb_planner.h"
#include <mutex>
#include <valarray>

#include <ros/ros.h>
#include <ros/assert.h>

#include "g2o_types/edge_acceleration.h"
#include "g2o_types/edge_deltaT.h"
// #include "g2o_types/edge_time_optimal.h"
#include "g2o_types/edge_velocity.h"
#include "g2o_types/edge_obstacle.h"
#include "g2o_types/edge_kinematic.h"

bool TebPlannerTest::initialized_ = false;

TebPlannerTest::TebPlannerTest(ConfigStruct& config, ObstacleVecPtr& obstacle_vec):
				config_(config),obstacle_vec_(obstacle_vec),optimized_(false)
{
	if(!initialized_)
	{
		RegisterG2OTypes();
		initialized_ = true;
	}
	optimizer_ = std::make_shared<g2o::SparseOptimizer>();
	TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
// 	linearSolver->setBlockOrdering(true);
	TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

	optimizer_->setAlgorithm(solver);
	optimizer_->setVerbose(false);
	optimizer_->initMultiThreading();
}

void TebPlannerTest::RegisterG2OTypes(void)
{
	g2o::Factory* factory = g2o::Factory::instance();
	factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
	factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);
	
	factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
	factory->registerType("EDGE_TIME_DIFF", new g2o::HyperGraphElementCreator<EdgeTimeDiff>);
	factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
}

// vel代表当前速度，同时用于输出下一时刻速度
void TebPlannerTest::plan(const PoseSE2& new_start, const PoseSE2& new_end, geometry_msgs::Twist& vel, double command_vel)
{
	bool new_path_flag = false;
	if(SizePoseVec() == 0)
	{
		AddPose(new_start, true);
		AddPose(new_end, true);
		//计算当前速度
		command_vel = (config_.max_start_vel_ + config_.max_vel_)/2;
		double current_vel = sqrt(vel.linear.x*vel.linear.x + vel.linear.y*vel.linear.y);
		double time_diff = (new_end.position()-new_start.position()).norm()/command_vel;
		AddTimeDiff(time_diff);
		new_path_flag = true;
	}
	else
	{
		PoseSE2 end_pose = pose_vec_.back()->pose();
		//如果目标点发生了改变，就需要重新计算初始轨迹
		if(new_end.position() != end_pose.position())
		{
			pose_vec_.clear();
			AddPose(new_start, true);
			AddPose(new_end, true);
			//计算当前速度
			command_vel = (config_.max_start_vel_ + config_.max_vel_)/2;
			double current_vel = sqrt(vel.linear.x*vel.linear.x + vel.linear.y*vel.linear.y);
			double time_diff = (new_end.position()-new_start.position()).norm()/command_vel;
			AddTimeDiff(time_diff);
			new_path_flag = true;
		}
	}
	AutoResize();

	bool success = BuildGraph();
	if(!success)
	{
		ClearGraph();
		return;
	}
	
// 	std::cout << optimizer_->edges().size() << std::endl;
	optimizer_->initializeOptimization();

	int iter = optimizer_->optimize(10);
	double trajectory_intervel = 0;
// 	if(iter)
// 	{
// 		std::cout << "The iter num is: " << iter << std::endl;
// 	}
// 	for(int i = 0; i < SizeTimeDiffVec(); ++i)
// 	{
// 		trajectory_intervel += TimeDiffVertex(i)->dt();
// 	}
// 	std::cout << "The whole time is: " << trajectory_intervel << std::endl;
	ClearGraph();
}

void TebPlannerTest::plan(const PlanStruct& global_plan, nav_msgs::Odometry& current_command)
{
// 	bool new_path_flag = false;
	PoseSE2 new_start;
	PoseSE2 new_end;
	
	new_start.x() = global_plan.current_pose.position.x;
	new_start.y() = global_plan.current_pose.position.y;
	new_start.theta() = tf::getYaw(global_plan.current_pose.orientation);
	
	new_end.x()   = global_plan.goal_pose.position.x;
	new_end.y()   = global_plan.goal_pose.position.y;
	new_end.theta() = tf::getYaw(global_plan.goal_pose.orientation);
	
	double current_vel = sqrt(pow(global_plan.current_vel.linear.x,2) + pow(global_plan.current_vel.linear.y,2));
	double goal_vel  = sqrt(pow(global_plan.goal_vel.linear.x,2) + pow(global_plan.current_vel.linear.y,2));
	double command_vel = 0;
	
	config_.max_start_omega_vel_ = global_plan.current_vel.angular.z;
	config_.max_end_omega_vel_ = global_plan.goal_vel.angular.z;
	//设置约束参数
	config_.max_start_vel_ = current_vel;
	config_.max_end_vel_   = goal_vel;
	
	if(config_.max_end_vel_ > 1)
	{
		command_vel = (config_.max_start_vel_ + config_.max_end_vel_)/2;
	}
	else
	{
		command_vel = (config_.max_start_vel_ + config_.max_vel_)/2;
	}
	
	if(SizePoseVec() == 0)
	{
		pose_vec_.clear();
		time_diff_vec_.clear();
		
		AddPose(new_start, true);
		AddPose(new_end, true);
		//计算当前速度

		double time_diff = (new_end.position()-new_start.position()).norm()/command_vel;
		AddTimeDiff(time_diff);
	}
	else if(SizePoseVec() == 1)
	{
		return;
	}
	else
	{
		PoseSE2 end_pose = pose_vec_.back()->pose();
		//如果目标点发生了改变，就需要重新计算初始轨迹
		if(new_end.position() != end_pose.position())
		{
			pose_vec_.clear();
			time_diff_vec_.clear();
			
			AddPose(new_start, true);
			AddPose(new_end, true);
			//计算当前速度
// 			command_vel = (config_.max_start_vel_ + config_.max_vel_)/2;
			double time_diff = (new_end.position()-new_start.position()).norm()/command_vel;
			AddTimeDiff(time_diff);
// 			new_path_flag = true;
			std::cout << "The end point has changed!" << std::endl;
		}
// 		std::cout << "The pose size is:" << SizePoseVec() << std::endl;
// 		std::cout << "The time diff size is: " << SizeTimeDiffVec() << std::endl;
		
		PruneTEBPoseAndTimediff(new_start);
	}
	
	AutoResize();
	
	if(SizePoseVec()>2)
	{
		bool success = BuildGraph();
		if(!success)
		{
			ClearGraph();
			return;
		}

	// 	std::cout << optimizer_->edges().size() << std::endl;
		optimizer_->initializeOptimization();
	// 
		static bool first_entry = true;
		if(first_entry)
		{
			int iter = optimizer_->optimize(10);
// 			first_entry = false;
		}
		double trajectory_intervel = 0;
	}
	
	NormalizeAngle();
	if(SizePoseVec() >= 3)
	{
		current_command.pose.pose.position.x = PoseVertex(4)->x();
		current_command.pose.pose.position.y = PoseVertex(4)->y();
// 		current_command.pose.pose.orientation = tf::createQuaternionMsgFromYaw(PoseVertex(4)->theta());
// 		current_command.twist.twist.linear.x  = PoseVelocity(2).linear.x;
// 		current_command.twist.twist.linear.y  = PoseVelocity(2).linear.y;
		current_command.twist.twist.angular.z = PoseVelocity(3).angular.z; 
	}
	ClearGraph();
}

void TebPlannerTest::PruneTEBPoseAndTimediff(const PoseSE2& new_start)
{
	if(SizePoseVec() < 3) return;
	//找到与new_start最近的点
	int dist_index = 0;
	double min_dist = 100;
	Eigen::Vector2d dist;
	
	for(int i=0; i<SizePoseVec(); ++i)
	{
		dist = PoseVertex(i)->position() - new_start.position();
		if(min_dist > dist.norm())
		{
			min_dist = dist.norm();
			dist_index = i;
		}
	}
	
	//判断最近点在当前点的哪个位置，根据位置的不同执行不同的策略
	double dist_angle   = atan2(dist(1), dist(0));
	double insect_angle = tfNormalizeAngle(dist_angle - PoseVertex(dist_index)->theta());
	if(fabs(insect_angle) < M_PI/2 && dist_index > 1)
	{
		//间隔点的距离
		double distance_point = (PoseVertex(dist_index)->position() - PoseVertex(dist_index-1)->position()).norm();
		double time_diff = TimeDiffVertex(dist_index-1)->dt() * min_dist / distance_point;
		//与前进方向同向,最近点位于前方，说明后方的点都可以删除了
		deletePoses(0,dist_index);
		deleteTimeDiffs(0,dist_index);
		//前方的最近点比较近，可以删除掉
		if(min_dist < config_.min_update_dist_ && SizePoseVec() > 2)
		{
			time_diff += TimeDiffVertex(0)->dt();
			ErasePose(0);
			EraseTimeDiff(0);
		}
		InsertTimeDiff(0,time_diff);
		InsertPose(0,new_start);
		PoseVertex(0)->setFixed(true);
	}
	if(fabs(insect_angle) >= M_PI/2 && dist_index > 1)
	{
		if(min_dist > config_.min_update_dist_)
		{
			double distance_point = (PoseVertex(dist_index)->position() - PoseVertex(dist_index-1)->position()).norm();
		    double time_diff = TimeDiffVertex(dist_index-1)->dt() * min_dist / distance_point;
			deletePoses(0,dist_index+1);
			deleteTimeDiffs(0,dist_index+1);
			InsertTimeDiff(0,time_diff);
			InsertPose(0,new_start);
			PoseVertex(0)->setFixed(true);
		}
	}
	std::cout << "Dist_index =:" << dist_index << std::endl;
}

//用于调整生成的轨迹
void TebPlannerTest::AutoResize(void)
{
	ROS_ASSERT(SizeTimeDiffVec() == SizePoseVec()-1);
	
	for(int i = 0; i<SizeTimeDiffVec()&&i<SizePoseVec()-1; ++i)
	{
		double timevel = TimeDiffVertex(i)->dt();
		if(timevel > config_.max_deta_t_ && (int)SizePoseVec() < config_.max_pose_num_)
		{
			TimeDiffVertex(i)->dt() = 0.5*timevel;
			PoseSE2 average = PoseSE2::average(PoseVertex(i)->pose(), PoseVertex(i+1)->pose());
			InsertPose(i+1, average);
			InsertTimeDiff(i+1, 0.5*timevel);
			--i;
		}
		else if(timevel < config_.min_deta_t_ && (i+1) < SizeTimeDiffVec())
		{
			TimeDiffVertex(i)->dt() = TimeDiffVertex(i)->dt() + TimeDiffVertex(i+1)->dt();
			ErasePose(i+1);
			EraseTimeDiff(i+1);
		}
	}
}

bool TebPlannerTest::BuildGraph(void)
{
	if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
	{
		ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
		return false;
	}
	AddTEBVertices();
	AddEdgeAcceleration();
	AddKinematic();
	AddEdgeVelocity();
	AddEdgeObstacle();
	AddEdgeTimeDiff();
	return true;
}

void TebPlannerTest::ClearGraph(void)
{
	optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
	optimizer_->clear();	
}

void TebPlannerTest::AddTEBVertices(void)
{
	// add vertices to graph
	unsigned int id_counter = 0; // used for vertices ids
	for (int i=0; i<SizePoseVec(); ++i)
	{
		PoseVertex(i)->setId(id_counter++);
		optimizer_->addVertex(PoseVertex(i));
		if (SizeTimeDiffVec()!=0 && i<SizeTimeDiffVec())
		{
			TimeDiffVertex(i)->setId(id_counter++);
			optimizer_->addVertex(TimeDiffVertex(i));
		}
	} 
}

void TebPlannerTest::CalTrajectory(hector_nav2::TrajectoryMsg& _trajectory)
{
	ros::Duration duration;
	ros::Duration whole_time;
	for(int i=0; i<SizePoseVec(); ++i)
	{
		geometry_msgs::Pose pose;
		pose.position.x = PoseVertex(i)->x();
		pose.position.y = PoseVertex(i)->y();
		pose.position.z = 0;
		pose.orientation = tf::createQuaternionMsgFromYaw(PoseVertex(i)->theta());
		
		geometry_msgs::Twist velocity;
		velocity = PoseVelocity(i);

		geometry_msgs::Twist acceleration;
		acceleration = PoseAcceleration(i);

		hector_nav2::TrajectoryPointMsg trajectory_point;
		trajectory_point.pose = pose;
		trajectory_point.velocity = velocity;
		trajectory_point.acceleration = acceleration;
		
		if(i < SizeTimeDiffVec())
		{
			whole_time += duration;
			duration.fromSec(TimeDiffVertex(i)->dt());
		}
		trajectory_point.time_from_start = whole_time;
		_trajectory.trajectory.push_back(trajectory_point);
	}
}

geometry_msgs::Twist TebPlannerTest::PoseVelocity(int i)
{
	ROS_ASSERT(i < (int)pose_vec_.size() && i >= 0);
	geometry_msgs::Twist velocity;
	if(i == SizePoseVec()-1)
	{
		if(i > 0)
		{
			velocity.linear.x  = config_.max_end_vel_;
			velocity.linear.y  = 0;
			velocity.angular.z = config_.max_end_omega_vel_;
			return velocity;
		}
		else
			return velocity;
	}
	if(i == 0)
	{
		velocity.linear.x  = config_.max_start_vel_;
		velocity.linear.y  = 0;
		velocity.angular.z = config_.max_start_omega_vel_;
		return velocity;
	}
	Eigen::Vector2d dist = (PoseVertex(i+1)->position() - PoseVertex(i)->position());
	double dist_orientation = atan2(dist[1],dist[0]);
	double dist_sign = fabs(tfNormalizeAngle(PoseVertex(i)->theta()-dist_orientation))>(M_PI/2)?-1:1;
	velocity.linear.x = dist.norm() / TimeDiffVertex(i)->dt();
	velocity.linear.y = 0/*dist[1]*/ / TimeDiffVertex(i)->dt();
	velocity.angular.z = (PoseVertex(i+1)->theta() - PoseVertex(i)->theta()) / TimeDiffVertex(i)->dt();
	return velocity;
}

geometry_msgs::Twist TebPlannerTest::PoseAcceleration(int i)
{
	ROS_ASSERT(i < (int)pose_vec_.size() && i >=0);
	geometry_msgs::Twist acceleration;
	if(i >= SizePoseVec()-2)
	{
		if(i > 1)
			return PoseAcceleration(i-1);
		else
			return acceleration;
	}
	acceleration.linear.x = (PoseVelocity(i+1).linear.x - PoseVelocity(i).linear.x) * 2 
							/ (TimeDiffVertex(i)->dt() + TimeDiffVertex(i+1)->dt());
	acceleration.linear.y = (PoseVelocity(i+1).linear.y - PoseVelocity(i).linear.y) * 2 
							/ (TimeDiffVertex(i)->dt() + TimeDiffVertex(i+1)->dt());
	acceleration.angular.z = (PoseVelocity(i+1).angular.z - PoseVelocity(i).angular.z) * 2
							 / (TimeDiffVertex(i+1)->dt() + TimeDiffVertex(i)->dt());
	return acceleration;
}

void TebPlannerTest::NormalizeAngle(void)
{
	for(int i=0; i<SizePoseVec(); ++i)
	{
		PoseVertex(i)->theta() = tfNormalizeAngle(PoseVertex(i)->theta());
	}
}

void TebPlannerTest::AddKinematic(void)
{
	//添加信息矩阵
	Eigen::Matrix2d information;
	information(0,0) = 3;
	information(1,1) = 3;
	//添加边到优化器
	int n = SizePoseVec();
	for(int i = 0; i < n-1; ++i)
	{
		EdgeKinematic* kinematic_edge = new EdgeKinematic;
		kinematic_edge->setVertex(0,PoseVertex(i));
		kinematic_edge->setVertex(1,PoseVertex(i+1));
		kinematic_edge->setInformation(information);
		optimizer_->addEdge(kinematic_edge);
	}
}

void TebPlannerTest::AddEdgeAcceleration(void)
{
	//添加信息矩阵
	Eigen::Matrix2d information;
	information(0,0) = 4;
	information(1,1) = 4;
	//添加边到优化器
	int n = SizePoseVec();
	for(int i = 0; i < n-2; ++i)
	{
		EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
		acceleration_edge->setVertex(0,PoseVertex(i));
		acceleration_edge->setVertex(1,PoseVertex(i+1));
		acceleration_edge->setVertex(2,PoseVertex(i+2));
		acceleration_edge->setVertex(3,TimeDiffVertex(i));
		acceleration_edge->setVertex(4,TimeDiffVertex(i+1));
		acceleration_edge->setInformation(information);
		acceleration_edge->setConfig(config_);
		optimizer_->addEdge(acceleration_edge);
	}
	for(int i = 3; i < n-3; ++i)
	{
		EdgeJerk* jerk_edge = new EdgeJerk;
		jerk_edge->setVertex(0,PoseVertex(i));
		jerk_edge->setVertex(1,PoseVertex(i+1));
		jerk_edge->setVertex(2,PoseVertex(i+2));
		jerk_edge->setVertex(3,PoseVertex(i+3));
		jerk_edge->setVertex(4,TimeDiffVertex(i));
		jerk_edge->setVertex(5,TimeDiffVertex(i+1));
		jerk_edge->setVertex(6,TimeDiffVertex(i+2));
		jerk_edge->setInformation(information);
		jerk_edge->setConfig(config_);
		optimizer_->addEdge(jerk_edge);
	}
	{
		EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
		acceleration_edge->setVertex(0,PoseVertex(0));
		acceleration_edge->setVertex(1,PoseVertex(1));
		acceleration_edge->setVertex(2,TimeDiffVertex(0));
		acceleration_edge->setInformation(7*information);
		acceleration_edge->setConfig(config_);
		optimizer_->addEdge(acceleration_edge);
	}
	{
		int n = SizePoseVec() - 1;
		EdgeAccelerationEnd* acceleration_edge = new EdgeAccelerationEnd;
		acceleration_edge->setVertex(0,PoseVertex(n-1));
		acceleration_edge->setVertex(1,PoseVertex(n));
		acceleration_edge->setVertex(2,TimeDiffVertex(n-1));
		acceleration_edge->setInformation(7*information);
		acceleration_edge->setConfig(config_);
		optimizer_->addEdge(acceleration_edge);
	}
}

void TebPlannerTest::AddEdgeVelocity(void)
{
	Eigen::Matrix2d information;
	information(0,0) = 1;
	information(1,1) = 1;
	
	//添加速度约束边到优化器
	int n = SizePoseVec();
	for(int i = 0; i < n-1; ++i)
	{
		EdgeVelocity* velocity_edge = new EdgeVelocity;
		velocity_edge->setVertex(0,PoseVertex(i));
		velocity_edge->setVertex(1,PoseVertex(i+1));
		velocity_edge->setVertex(2,TimeDiffVertex(i));
		velocity_edge->setInformation(information);
		velocity_edge->setConfig(config_);
		optimizer_->addEdge(velocity_edge);
	}
	
// // 	添加初始速度约束边到优化器
// 	EdgeStartVelocity* start_velocity_edge = new EdgeStartVelocity;
// 	start_velocity_edge->setVertex(0, PoseVertex(0));
// 	start_velocity_edge->setVertex(1, PoseVertex(1));
// 	start_velocity_edge->setVertex(2, TimeDiffVertex(0));
// 	start_velocity_edge->setInformation(10*information);
// 	start_velocity_edge->setConfig(config_);
// 	optimizer_->addEdge(start_velocity_edge);
// 	
// //  添加末端速度约束边到优化器
// 	EdgeEndVelocity* end_velocity_edge = new EdgeEndVelocity;
// 	end_velocity_edge->setVertex(0, PoseVertex(n-2));
// 	end_velocity_edge->setVertex(1, PoseVertex(n-1));
// 	end_velocity_edge->setVertex(2, TimeDiffVertex(n-2));
// 	end_velocity_edge->setInformation(10*information);
// 	end_velocity_edge->setConfig(config_);
// 	optimizer_->addEdge(end_velocity_edge);
}

void TebPlannerTest::AddEdgeObstacle(void)
{
	if(obstacle_vec_->size() == 0)
		return;
	//寻找最近的点，添加边到优化器
	for(int i = 0; i < obstacle_vec_->size(); ++i)
	{
		for(int j = 0; j < SizePoseVec(); ++j)
		{
			Eigen::Vector2d dist = PoseVertex(j)->position() - obstacle_vec_->at(i).getPosition();
			if(dist.norm() < config_.min_obstacle_dist_)
			{
				Eigen::Matrix<double,1,1> information;
				information(0,0) = 10;
				EdgeObstacle* obstacle_edge = new EdgeObstacle;
				obstacle_edge->setVertex(0,PoseVertex(j));
				obstacle_edge->setObstacle(obstacle_vec_->at(i));
				obstacle_edge->setConfig(config_);
				obstacle_edge->setInformation(information);
				optimizer_->addEdge(obstacle_edge);
				std::cout << "There exist obstacle!" << dist.norm() << std::endl;
			}
		}
	}
}

void TebPlannerTest::AddEdgeTimeDiff(void)
{
	Eigen::Matrix<double,1,1> information;
	information(0,0) = 4;
	int n = SizeTimeDiffVec();
	for(int i = 0; i < n; ++i)
	{
// 		EdgeTimeOptimal* time_diff_edge = new EdgeTimeOptimal;
		EdgeTimeDiff* time_diff_edge = new EdgeTimeDiff;
		time_diff_edge->setVertex(0,TimeDiffVertex(i));
		time_diff_edge->setInformation(information);
		optimizer_->addEdge(time_diff_edge);
	}
}

VertexPose* TebPlannerTest::PoseVertex(int index)
{
	ROS_ASSERT(index < (int)pose_vec_.size() && i >= 0);
	return pose_vec_.at(index);
}
 
VertexTimeDiff* TebPlannerTest::TimeDiffVertex(int index)
{
	ROS_ASSERT(index < (int)pose_vec_.size() && i >= 0);
	return time_diff_vec_.at(index);
}

int TebPlannerTest::SizePoseVec(void)
{
	return (int)pose_vec_.size();
}

int TebPlannerTest::SizeTimeDiffVec(void)
{
	return (int)time_diff_vec_.size();
}

void TebPlannerTest::AddPose(const PoseSE2& pose, bool fixed)
{
	VertexPose* pose_vertex = new VertexPose(pose,fixed);
	pose_vec_.push_back(pose_vertex);
}

void TebPlannerTest::AddTimeDiff(double dt)
{
	VertexTimeDiff* time_diff = new VertexTimeDiff(dt);
	time_diff_vec_.push_back(time_diff);
}

void TebPlannerTest::InsertPose(int index, const PoseSE2& pose)
{
	ROS_ASSERT(index < (int)pose_vec_.size() && i >= 0); 
	VertexPose* pose_vertex = new VertexPose(pose);
	pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TebPlannerTest::InsertTimeDiff(int index, double dt)
{
	ROS_ASSERT(index < (int)time_diff_vec_.size() && i >= 0);
	VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
	time_diff_vec_.insert(time_diff_vec_.begin()+index, timediff_vertex);
}

void TebPlannerTest::ErasePose(int index)
{
	ROS_ASSERT(index < (int)pose_vec_.size() && i >= 0);
	delete pose_vec_.at(index);
	pose_vec_.erase(pose_vec_.begin()+index);
}

void TebPlannerTest::EraseTimeDiff(int index)
{
	ROS_ASSERT(index < (int)time_diff_vec_.size() && i >= 0);
	delete time_diff_vec_.at(index);
	time_diff_vec_.erase(time_diff_vec_.begin()+index);
}

void TebPlannerTest::deletePoses(int index, int number)
{
	ROS_ASSERT(index+number<=(int)pose_vec_.size());
	for (int i = index; i<index+number; ++i)
		delete pose_vec_.at(i);
	pose_vec_.erase(pose_vec_.begin()+index, pose_vec_.begin()+index+number);
}

void TebPlannerTest::deleteTimeDiffs(int index, int number)
{
	ROS_ASSERT(index+number<=(int)time_diff_vec_.size());
	for (int i = index; i<index+number; ++i)
		delete time_diff_vec_.at(i);
	time_diff_vec_.erase(time_diff_vec_.begin()+index, time_diff_vec_.begin()+index+number);
}