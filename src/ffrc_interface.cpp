#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <ros/ros.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ffrc_interface/ffrc.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/config.h>

#include <iostream>
#include <thread>

#include "ffrc_interface/ffrc_interface.h"

namespace ffrc_interface
{
  FFRCInterface::FFRCInterface(const ros::NodeHandle &nh, int val)
      : nh_(nh), name_("FFRCInterface"), val_(val)
  {
    ROS_INFO_STREAM_NAMED("ffrc_interface", "Iniciando FFRC OMPL interface using ROS parameters");
    // loadPlannerConfigurations();
    // loadConstraintSamplers();
  }

  bool FFRCInterface::isStateValid(const ompl::base::State *state)
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    moveit::core::RobotState robot_state = pc_->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(req_.group_name);
    double *val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    robot_state.setJointGroupPositions(joint_model_group, val);
    for (auto i = 0; i < dof_; i++)
    {
      std::cout << robot_state.getJointPositions(joint_model_group->getJointModels().at(i))[0] << ", ";
    }
    // check collision avoidance
    std::cout << "validando estados \n";
    collision_result.clear();
    this->pc_->checkCollision(collision_request, collision_result, robot_state);
    std::cout << "lo colicion es: " << collision_result.collision << "\n";
    // collision=true, estado invalido, toncs false
    return !collision_result.collision;
  }

  bool FFRCInterface::solve(const planning_scene::PlanningSceneConstPtr &planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            moveit_msgs::MotionPlanDetailedResponse &res)
  {
    // Load the planner-specific parameters
    nh_.param("num_steps", num_steps_, 10);
    pc_ = planning_scene;
    req_ = req;
    ros::WallTime start_time = ros::WallTime::now();
    moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
    moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model));
    *start_state = planning_scene->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group = start_state->getJointModelGroup(req.group_name);
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    dof_ = joint_names.size();

    std::vector<double> start_joint_values;
    start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

    std::vector<double> goal_joint_values;
    // goal_joint_values.reserve(goal_joint_constraint.size());
    for (const auto &constraint : req_.goal_constraints.at(0).joint_constraints)
    {
      goal_joint_values.push_back(constraint.position);
    }

    // aqui meto mano

    // tomar el numero de juntas, revisar los limites articulares para cada junta
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(dof_));
    space->setName("C-space");
    // set the bounds for the state space
    ompl::base::RealVectorBounds bounds(dof_);
    for (auto i = 0; i < dof_; i++)
    {
      bounds.setLow(i, joint_model_group->getJointModels().at(i)->getVariableBounds().at(0).min_position_);
      bounds.setHigh(i, joint_model_group->getJointModels().at(i)->getVariableBounds().at(0).max_position_);
    }
    space->setBounds(bounds);

    // construct an instance of space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    si->printSettings(std::cout);

    // set state validity checking for this space
    // this hace referecnia a esta clase
    std::function<bool(const ompl::base::State *state)> f = std::bind(&FFRCInterface::isStateValid, this, std::placeholders::_1);
    si->setStateValidityChecker(f);

    // create a test state
    ompl::base::ScopedState<> stado(space);
    stado = std::vector<double>{0, -0.122173, 0.663225, 0, 1.64061, 0};
    std::cout << "prueba del bind ese raro \n"
              << si->getStateValidityChecker()->isValid(stado.get()) << "\n";

    // start state
    ompl::base::ScopedState<> start(space);
    start = start_joint_values;
    // goal state
    ompl::base::ScopedState<> goal(space);
    goal = goal_joint_values;

    // create a problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    // auto planner(std::make_shared<og::RRT>(si));
    auto planner(std::make_shared<ompl::geometric::FFRC>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning delay
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(5.0);

    if (solved)
    {
      // get the goal representation from the problem definition (not the same as the goal state)
      // and inquire about the found path
      std::cout << "Found solution: \n";

      // print the path to screen
      ompl::geometric::PathGeometric *path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
      path->print(std::cout);
      std::cout << path->getStateCount() << "\n";

      trajectory_msgs::JointTrajectory joint_trajectory;
      joint_trajectory.header = req.start_state.joint_state.header;
      joint_trajectory.joint_names = joint_names;
      joint_trajectory.points.resize(path->getStateCount());

      double *n;
      for (size_t j = 0; j < path->getStateCount(); j++)
      {
        n = path->getState(j)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        std::vector<double> joint_vals;
        for (auto i = 0; i < dof_; i++)
        {
          joint_vals.push_back(n[i]);
        }
        joint_trajectory.points[j].positions = joint_vals;
        joint_trajectory.points[j].time_from_start = ros::Duration(double(j));
      }

      std::cout << joint_trajectory << "\n";

      // ==================== feed the response
      res.trajectory.resize(1);
      res.trajectory[0].joint_trajectory = joint_trajectory;

      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());
      res.group_name = req.group_name;
      res.trajectory_start.joint_state.name = joint_names;
      res.trajectory_start.joint_state.position = start_joint_values;

      ompl::base::PlannerData pd(si);
      planner->getPlannerData(pd);
      std::cout << pd.numVertices() << "\n";
      std::cout << "printing los vertices de la trayectoria \n";
      // for (size_t i = 0; i < pd.numVertices(); i++)
      // {
      //   double *val = pd.getVertex(i).getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      //   ompl::base::ScopedState<> stt(space);
      //   stt = pd.getVertex(i).getState();
      //   stt.print();
      // }
    }
    else
    {
      std::cout << "No solution found" << std::endl;
      res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    }

    // hasta aqui el caos

    // ==================== Interpolation
    // trajectory_msgs::JointTrajectory joint_trajectory;
    // interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

    // ==================== feed the response
    // res.trajectory.resize(1);
    // res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
    // res.trajectory[0].joint_trajectory.joint_names = joint_names;
    // res.trajectory[0].joint_trajectory = joint_trajectory;

    // res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    // res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

    // res.group_name = req.group_name;
    // res.trajectory_start.joint_state.name = joint_names;
    // res.trajectory_start.joint_state.position = start_joint_values;

    return true;
  }

  void FFRCInterface::interpolate(const std::vector<std::string> &joint_names, moveit::core::RobotStatePtr &rob_state,
                                  const moveit::core::JointModelGroup *joint_model_group,
                                  const std::vector<double> &start_joint_vals, const std::vector<double> &goal_joint_vals,
                                  trajectory_msgs::JointTrajectory &joint_trajectory)
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

} // namespace ffrc_interface
