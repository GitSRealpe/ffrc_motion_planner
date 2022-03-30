#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// #include <ompl/geometric/planners/rrt/RRT.h>
#include <ffrc_interface/ffrc.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

char title[] = "Sampling";
int delay = 1;

bool isStateValid(const ob::State *state)
{
    // std::cout << "validando, beep boop\n";
    double *val = state->as<ob::RealVectorStateSpace::StateType>()->values;
    for (int i = 0; i < 6; i++)
    {
        std::cout << val[i] << ", ";
    }

    std::cout << "\n";

    return true;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    space->setName("Spacio6d");
    // auto sampler(space->allocDefaultStateSampler());

    // set the bounds for the state space
    ob::RealVectorBounds bounds(6);

    for (size_t i = 0; i < 6; i++)
    {
        bounds.setLow(i, -3.14);
        bounds.setHigh(i, 3.14);
    }

    space->setBounds(bounds);

    // construct an instance of space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    // si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();
    do
    {
        start.random();
    } while (!isStateValid(start.get()));

    // create a random goal state
    ob::ScopedState<> goal(space);
    do
    {
        goal.random();
    } while (!isStateValid(goal.get()));

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    // auto planner(std::make_shared<og::RRT>(si));
    auto planner(std::make_shared<og::FFRC>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning delay
    ob::PlannerStatus solved = planner->ob::Planner::solve(60.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);

        ob::PlannerData pd(si);
        planner->getPlannerData(pd);
        std::cout << pd.numVertices() << std::endl;

        // for (size_t i = 0; i < pd.numVertices(); i++)
        // {
        //     double *val = pd.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>()->values;
        // }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    // si->printProperties(std::cout);
    // si->printSettings(std::cout << "\n");
    // std::cout<<"\n";

    std::cout << "\n";
    // space->diagram(std::cout);
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION_VALUE << std::endl;

    plan();

    cv::waitKey(0);

    return 0;
}