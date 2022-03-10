#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <ros/ros.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "ffrc_interface/ffrc_interface.h"

namespace ffrc_interface
{
FFRCInterface::FFRCInterface(const ros::NodeHandle& nh) : nh_(nh), name_("FFRCInterface")
{
}

bool FFRCInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::MotionPlanDetailedResponse& res)
{
  // Load the planner-specific parameters
  nh_.param("num_steps", num_steps_, 10);

  ros::WallTime start_time = ros::WallTime::now();
  moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model));
  *start_state = planning_scene->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  dof_ = joint_names.size();
  std::vector<double> start_joint_values;
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

  // This planner only supports one goal constraint in the request
  const std::vector<moveit_msgs::Constraints>& goal_constraints = req.goal_constraints;
  const std::vector<moveit_msgs::JointConstraint>& goal_joint_constraint = goal_constraints[0].joint_constraints;

  std::vector<double> goal_joint_values;
  goal_joint_values.reserve(goal_joint_constraint.size());

  for (const auto& constraint : goal_joint_constraint)
  {
    goal_joint_values.push_back(constraint.position);
  }

  // ==================== Interpolation
  trajectory_msgs::JointTrajectory joint_trajectory;
  interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

  // ==================== feed the response
  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
  res.trajectory[0].joint_trajectory = joint_trajectory;

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  res.group_name = req.group_name;
  res.trajectory_start.joint_state.name = joint_names;
  res.trajectory_start.joint_state.position = start_joint_values;

  return true;
}

void FFRCInterface::interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
                                const moveit::core::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::JointTrajectory& joint_trajectory)
{
  joint_trajectory.points.resize(num_steps_ + 1);

  std::vector<double> dt_vector;
  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
    double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
    dt_vector.push_back(dt);
  }

  for (int step = 0; step <= num_steps_; ++step)
  {
    std::vector<double> joint_values;
    for (int k = 0; k < dof_; ++k)
    {
      double joint_value = start_joint_vals[k] + step * dt_vector[k];
      joint_values.push_back(joint_value);
    }
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    rob_state->update();

    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points[step].positions = joint_values;
  }
}

}  // namespace ffrc_interface
