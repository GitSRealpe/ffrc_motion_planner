#include "ffrc_interface/ffrc.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::FFRC::FFRC(const base::SpaceInformationPtr &si)
    : base::Planner(si, "FFRC")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &FFRC::setRange, &FFRC::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &FFRC::setGoalBias, &FFRC::getGoalBias, "0.:.05:1.");
}

ompl::geometric::FFRC::~FFRC()
{
    freeMemory();
}

void ompl::geometric::FFRC::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::FFRC::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    std::cout << "distancia maxima entre estados " << maxDistance_ << std::endl;

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return distanceFunction(a, b); });
}

void ompl::geometric::FFRC::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::FFRC::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        std::cout << "Se agrego el start state" << std::endl;
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> structura;

    while (!ptc)
    {
        // nn_->list(structura);
        // structura.at(0)->state.as<real no se que;

        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        std::cout << "sampleado             " << dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << ", " << dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;

        std::cout << "nearest en el tree    " << nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << ", " << nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        std::cout << "distance " << d << std::endl;
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
            std::cout << "nearest interpolado " << dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << ", " << dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
        }

        std::cout << std::endl;

        if (si_->checkMotion(nmotion->state, dstate))
        {

            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            nn_->add(motion);

            nmotion = motion;

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::FFRC::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
