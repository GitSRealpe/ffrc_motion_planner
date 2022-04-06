#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ffrc_interface/ffrc_planning_context.h"
#include "ffrc_interface/ffrc_interface.h"

namespace ffrc_interface
{
  FFRCPlanningContext::FFRCPlanningContext(const std::string &context_name, const std::string &ns,
                                           const std::string &group_name, const moveit::core::RobotModelConstPtr &model)
      : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
  {
    std::cout << "creando la interface \n";
    ffrc_interface_ = FFRCInterfacePtr(new FFRCInterface(ros::NodeHandle(ns), 1));
  }

  bool FFRCPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
  {
    std::cout << "iniciando el contexto \n";
    std::cout << getPlanningScene()->getPlanningFrame() << "\n";
    moveit_msgs::MotionPlanDetailedResponse res_msg;
    // renombrar esta funcion?
    bool ffrc_solved = ffrc_interface_->solve(planning_scene_, request_, res_msg);

    if (ffrc_solved)
    {
      res.trajectory_.resize(1);
      res.trajectory_[0] =
          robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

      moveit::core::RobotState start_state(robot_model_);
      moveit::core::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);

      res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
      res.description_.push_back("plan");
      res.processing_time_ = res_msg.processing_time;
      res.error_code_ = res_msg.error_code;

      return true;
    }

    res.error_code_ = res_msg.error_code;
    return false;
  };

  bool FFRCPlanningContext::solve(planning_interface::MotionPlanResponse &res)
  {
    planning_interface::MotionPlanDetailedResponse res_detailed;
    bool planning_success = solve(res_detailed);

    res.error_code_ = res_detailed.error_code_;

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }

    return planning_success;
  }

  bool FFRCPlanningContext::terminate()
  {
    return true;
  }

  void FFRCPlanningContext::clear()
  {
    // This planner has no state, so has nothing to clear
  }

} // namespace ffrc_interface
