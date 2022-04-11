#pragma once

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/config.h>

// int dof_;
// planning_scene::PlanningSceneConstPtr pc_;
// planning_interface::MotionPlanRequest req_;
namespace ffrc_interface
{

  MOVEIT_CLASS_FORWARD(FFRCInterface);

  class FFRCInterface
  {
  public:
    FFRCInterface(const ros::NodeHandle &nh = ros::NodeHandle("~"), const int val = 0);

    bool solve(const planning_scene::PlanningSceneConstPtr &planning_scene,
               const planning_interface::MotionPlanRequest &req, moveit_msgs::MotionPlanDetailedResponse &res);
    // bool isStateValid(const ompl::base::State *state);
    bool isStateValid(const ompl::base::State *state);

  protected:
    ros::NodeHandle nh_;
    std::string name_;
    int num_steps_;
    int val_;
    int dof_;
    planning_scene::PlanningSceneConstPtr pc_;
    planning_interface::MotionPlanRequest req_;

  private:
    void interpolate(const std::vector<std::string> &joint_names, moveit::core::RobotStatePtr &robot_state,
                     const moveit::core::JointModelGroup *joint_model_group, const std::vector<double> &start_joint_vals,
                     const std::vector<double> &goal_joint_vals, trajectory_msgs::JointTrajectory &joint_trajectory);
  };
} // namespace ffrc_interface
