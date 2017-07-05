#ifndef __TEB_PLANNER_H__
#define __TEB_PLANNER_H__

#include <memory>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "g2o_types/vertex_pose.h"
#include "g2o_types/vertex_timediff.h"

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "hector_nav2/TrajectoryMsg.h"
#include "teb_config.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
typedef std::shared_ptr<std::vector<ObstacleStruct>> ObstacleVecPtr; 

struct PlanStruct
{
public:
	geometry_msgs::Pose  current_pose;
	geometry_msgs::Twist current_vel;
	geometry_msgs::Pose  goal_pose;
	geometry_msgs::Twist goal_vel;
};

class TebPlannerTest
{
public:
	TebPlannerTest()
	{}
	TebPlannerTest(ConfigStruct& config, ObstacleVecPtr& obstacle_vec);
	virtual ~TebPlannerTest()
	{}
	
	void plan(const PoseSE2& new_start, const PoseSE2& new_end, geometry_msgs::Twist& vel, double command_vel);
	void plan(const PlanStruct& global_plan, nav_msgs::Odometry& current_command);
	
	VertexPose* PoseVertex(int index);
	VertexTimeDiff* TimeDiffVertex(int index);
	int  SizePoseVec(void);
	int  SizeTimeDiffVec(void);
	void CalTrajectory(hector_nav2::TrajectoryMsg& _trajectory);
private:
	void RegisterG2OTypes(void);
	bool BuildGraph(void);
	void ClearGraph(void);
	
	void PruneTEBPoseAndTimediff(const PoseSE2& new_start);
	
	void AutoResize(void);
	void AddTEBVertices(void);
	void NormalizeAngle(void);
	void AddKinematic(void);
	void AddEdgeAcceleration(void);
	void AddEdgeVelocity(void);
	void AddEdgeObstacle(void);
	void AddEdgeTimeDiff(void);
	
	void AddPose(const PoseSE2& pose, bool fixed);
	void AddTimeDiff(double dt);
	void InsertPose(int index, const PoseSE2& pose);
	void InsertTimeDiff(int index, double dt);
	void ErasePose(int index);
	void EraseTimeDiff(int index);
	void deletePoses(int index, int number);
	void deleteTimeDiffs(int index, int number);
	
	geometry_msgs::Twist PoseVelocity(int i);
	geometry_msgs::Twist PoseAcceleration(int i);
	
	std::shared_ptr<g2o::SparseOptimizer> optimizer_;              //优化器 
	ConfigStruct   config_;
	ObstacleVecPtr obstacle_vec_;
	
	static bool initialized_;                     //用于标记是否应该初始化
	bool optimized_; 
	
	std::vector<VertexPose*>     pose_vec_;        //用于存放状态节点
	std::vector<VertexTimeDiff*> time_diff_vec_;   //用于存放时间间隔节点
	
  
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
};

#endif