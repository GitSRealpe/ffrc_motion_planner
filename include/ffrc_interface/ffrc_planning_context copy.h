#ifndef FFRC_PLANNING_CONTEXT_H
#define FFRC_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include "ffrc_interface/ffrc_interface.h"

namespace ffrc_interface
{
MOVEIT_CLASS_FORWARD(FFRCPlanningContext);

class FFRCPlanningContext : public planning_interface::PlanningContext
{
public:
  FFRCPlanningContext(const std::string& name, const std::string& ns, const std::string& group,
                      const moveit::core::RobotModelConstPtr& model);
  ~FFRCPlanningContext() override
  {
  }

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  FFRCInterfacePtr ffrc_interface_;
};

}  // namespace ffrc_interface

#endif  // FFRC_PLANNING_CONTEXT_H
