#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <class_loader/class_loader.hpp>
#include "ffrc_interface/ffrc_planning_context.h"

namespace ffrc_interface
{
class FFRCPlannerManager : public planning_interface::PlannerManager
{
public:
  FFRCPlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override
  {
    for (const std::string& gpName : model->getJointModelGroupNames())
    {
      planning_contexts_[gpName] =
          FFRCPlanningContextPtr(new FFRCPlanningContext("ffrc_planning_context", ns, gpName, model));
    }
    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "FFRC";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("ffrc");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // retrieve and configure existing context
    const FFRCPlanningContextPtr& context = planning_contexts_.at(req.group_name);
    ROS_INFO_STREAM_NAMED("ffrc_planner_manager", "===>>> context is made ");

    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
  }

protected:
  std::map<std::string, FFRCPlanningContextPtr> planning_contexts_;
};

}  // namespace ffrc_interface

// register the FFRCPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(ffrc_interface::FFRCPlannerManager, planning_interface::PlannerManager);
