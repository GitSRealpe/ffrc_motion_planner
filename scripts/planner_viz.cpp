#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// #include <ompl/geometric/planners/rrt/RRTConnect.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/prm/PRM.h>
#include <ffrc_interface/ffrc.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>

#define h 800
#define w 1000

namespace ob = ompl::base;
namespace og = ompl::geometric;

cv::Mat atom_image = cv::Mat::zeros(h, w, CV_8UC3);
std::vector<std::vector<cv::Point>> contours;
char title[] = "Sampling";
int delay = 1;

void createWorld()
{
    cv::rectangle(atom_image, cv::Point((w / 2 - 200), (h / 2 - 200)), cv::Point((w / 2), (h / 2)), cv::Scalar(107, 183, 189), cv::FILLED);
    cv::rectangle(atom_image, cv::Point((w / 2 - 10), (h / 2 - 10)), cv::Point((w / 2 + 190), (h / 2 + 190)), cv::Scalar(107, 183, 189), cv::FILLED);
    cv::Mat src_gray;
    cv::cvtColor(atom_image, src_gray, cv::COLOR_BGR2GRAY);

    cv::findContours(src_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(atom_image, contours, -1, cv::Scalar(255, 255, 255), 2);
    std::cout << contours.size() << "\n";
}

void pintarWorld()
{
    cv::circle(atom_image, cv::Point((w / 2), (h / 2)), 200 / 32, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    cv::arrowedLine(atom_image, cv::Point((w / 2), (h / 2)), cv::Point((w / 2) + 100, (h / 2)), cv::Scalar(0, 0, 255));
    cv::arrowedLine(atom_image, cv::Point((w / 2), (h / 2)), cv::Point((w / 2), (h / 2) + 100), cv::Scalar(0, 255, 0));
    cv::imshow(title, atom_image);
    cv::moveWindow(title, 0, 200);
    // cv::waitKey(0);
}

bool isStateValid(const ob::State *state)
{
    // std::cout << "validando, beep boop\n";
    double *val = state->as<ob::RealVectorStateSpace::StateType>()->values;
    // std::cout<<val[0]<<","<<val[1]<<"\n";

    cv::Scalar color;
    cv::Point punto;
    int test;
    punto = cv::Point((val[0] * 10 + w / 2), (val[1] * 10 + h / 2));
    test = cv::pointPolygonTest(contours[0], punto, false);
    if (test > 0)
    {
        // std::cout << "adentro\n";
        cv::circle(atom_image, punto, 200 / 32, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
        cv::imshow(title, atom_image);
        cv::waitKey(delay);
        return false;
    }
    else
    {
        // std::cout << "afuera\n";
        cv::circle(atom_image, punto, 200 / 32, cv::Scalar(189, 183, 107), cv::FILLED, cv::LINE_8);
        cv::imshow(title, atom_image);
        cv::waitKey(delay);
        return true;
    }

    return true;
}

void plan()
{
    createWorld();
    pintarWorld();
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setName("Spacio2d");
    // auto sampler(space->allocDefaultStateSampler());

    // set the bounds for the state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-40);
    bounds.setHigh(40);
    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    // si->setup();

    cv::Point punto;
    std::vector<double> val;
    // create a random start state
    ob::ScopedState<> start(space);
    start.random();
    do
    {
        start.random();
    } while (!isStateValid(start.get()));
    val = start.reals();
    punto = cv::Point((val[0] * 10 + w / 2), (val[1] * 10 + h / 2));
    cv::circle(atom_image, punto, 200 / 8, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_8);

    // create a random goal state
    ob::ScopedState<> goal(space);
    do
    {
        goal.random();
    } while (!isStateValid(goal.get()));
    val = goal.reals();
    punto = cv::Point((val[0] * 10 + w / 2), (val[1] * 10 + h / 2));
    cv::circle(atom_image, punto, 200 / 8, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    // pintar start y goal
    cv::imshow(title, atom_image);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    // auto planner(std::make_shared<og::RRTConnect>(si));
    // auto planner(std::make_shared<og::PRM>(si));
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

        for (size_t i = 0; i < pd.numVertices(); i++)
        {
            double *val = pd.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>()->values;
            cv::Point punto;
            punto = cv::Point((val[0] * 10 + w / 2), (val[1] * 10 + h / 2));
            cv::circle(atom_image, punto, 200 / 16, cv::Scalar(130, 0, 75), cv::FILLED, cv::LINE_8);
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    // si->printProperties(std::cout);
    // si->printSettings(std::cout << "\n");
    // std::cout<<"\n";
    // pintarWorld();
    cv::imshow(title, atom_image);
    cv::waitKey(5000);

    std::cout << "\n";
    // space->diagram(std::cout);
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION_VALUE << std::endl;

    plan();
    // atom_image = cv::Mat::zeros(h, w, CV_8UC3);
    // plan();
    // atom_image = cv::Mat::zeros(h, w, CV_8UC3);
    // plan();

    cv::waitKey(0);

    return 0;
}