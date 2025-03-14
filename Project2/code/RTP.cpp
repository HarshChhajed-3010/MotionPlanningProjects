#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "RTP.h"

ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "RTPintermediate" : "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RTP::setIntermediateStates, &RTP::getIntermediateStates, "0,1");

    addIntermediateStates_ = addIntermediateStates;
    maxDistance_ = 0.1;  // Added default maximum extension length
}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

double ompl::geometric::RTP::distanceFunction(const Motion *a, const Motion *b) const
{
    return si_->distance(a->state, b->state);
}

void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // Create the NN structure if it does not exist
    if (!nn_)
    {
        // Using NearestNeighborsLinear since RTP only needs random node selection
        nn_.reset(new NearestNeighborsLinear<Motion*>());
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
        {
            return distanceFunction(a, b);
        });
    }
}

void ompl::geometric::RTP::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: No valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    while (!ptc)
    {
        base::State *rstate = si_->allocState();
        if ((goal_s != nullptr) && (rng_.uniform01() < getGoalBias()) && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Random node selection (core RTP behavior)
        std::vector<Motion*> motions;
        nn_->list(motions); 
        if (motions.empty())
        {
            si_->freeState(rstate);//
            continue;
        }
        size_t idx = rng_.uniformInt(0, motions.size() - 1);
        Motion *nmotion = motions[idx];

        double d = si_->distance(nmotion->state, rstate);
        base::State *newState = rstate;
        base::State *interpolated = si_->allocState();
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, interpolated);
            newState = interpolated;
        }

        if (si_->checkMotion(nmotion->state, newState))
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, newState);
            motion->parent = nmotion;
            nn_->add(motion);

            double dist = 0.0;
            bool satisfied = goal->isSatisfied(motion->state, &dist);
            if (satisfied)
            {
                approxdif = dist;
                solution = motion;
                si_->freeState(interpolated);
                si_->freeState(rstate);
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }

        si_->freeState(interpolated);
        si_->freeState(rstate);
    }

    bool solved = false;
    bool approximate = false;
    if (!solution)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution)
    {
        lastGoalMotion_ = solution;
        std::vector<Motion *> mpath;
        while (solution)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}