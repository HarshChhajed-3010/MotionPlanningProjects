#include "ompl/geometric/planners/rrt/RRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
  
ompl::geometric::RRT::RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates) // constructor of the class  RRT which is derived from base::Planner class   
   : base::Planner(si, addIntermediateStates ? "RRTintermediate" : "RRT") // calling the constructor of the base class Planner , si is the space information and addIntermediateStates is the boolean value.
 {
     specs_.approximateSolutions = true; // setting the approximateSolutions to true, why? because we are using the approximate solution in the solve function.
     specs_.directed = true; // setting the directed to true, why? because we are using the directed graph in the solve function. where is the solve function? in the base::Planner class.
  
     Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000."); // declaring the parameter range, what is the range? it is the maximum distance between the two states.
     Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1."); // declaring the parameter goal_bias, what is the goal_bias? it is the probability of selecting the goal state as the random state.
     Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates, 
                                 "0,1"); // declaring the parameter intermediate_states, what is the intermediate_states? it is the boolean value which tells whether to add the intermediate states or not.
  
     addIntermediateStates_ = addIntermediateStates; // setting the addIntermediateStates_ to addIntermediateStates, why? because we are using the addIntermediateStates in the solve function.
 }
  
ompl::geometric::RRT::~RRT()
 {
     freeMemory(); // calling the freeMemory function. what is the freeMemory function? it is the function which frees the memory of the planner.
 }
  
void ompl::geometric::RRT::clear()
 {
     Planner::clear(); // calling the clear function of the base::Planner class. what is the clear function? it is the function which clears the planner.
     sampler_.reset(); // resetting the sampler_, why? because we are using the sampler_ in the solve function.
     freeMemory();  // calling the freeMemory function. what is the freeMemory function? it is the function which frees the memory.
     if (nn_) // checking if the nn_ is not null pointer. what is the nn_? it is the nearest neighbor data structure.
         nn_->clear(); // clearing the nn_, why? because we are using the nn_ in the solve function. 
     lastGoalMotion_ = nullptr; // setting the lastGoalMotion_ to nullptr.
 }
  
void ompl::geometric::RRT::setup() // setup function of the class RRT which is derived from base::Planner class.
 {
     Planner::setup(); // calling the setup function of the base::Planner class.
     tools::SelfConfig sc(si_, getName()); // creating the object of the SelfConfig class. what is the SelfConfig class? it is the class which is used to configure the planner. sc is the object of the SelfConfig class. getName() is the function which returns the name of the planner.
     sc.configurePlannerRange(maxDistance_); // configuring the planner range, what is the maxDistance_? it is the maximum distance between the two states. value of maxDistance_ is set by the user. sc is the object of the SelfConfig class. selfconfig can be found in the tools/config/SelfConfig.h file.
  
     if (!nn_) // checking if the nn_ is null pointer. nn_ is the nearest neighbor data structure. if its true then create the nearest neighbor data structure.,  if false then do nothing.
         nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this)); // creating the nearest neighbor data structure. what is the Motion? its the class which is used to store the state and parent of the state.
     nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); }); // setting the distance function of the nearest neighbor data structure. what is the distanceFunction? it is the function which calculates the distance between the two states.
 }
  
void ompl::geometric::RRT::freeMemory() // freeMemory function of the class RRT which is derived from base::Planner class. why we need it ? because we need to free the memory of the planner.
 {
     if (nn_) // checking if the nn_ is not null pointer. when it is true then do the following., not a null ptr means ? it means that the nearest neighbor data structure is created. that means that we have a memory allocated for the nearest neighbor data structure.
     {
         std::vector<Motion *> motions; // creating the vector of the Motion class. what is the Motion class? it is the class which is used to store the state and parent of the state. state meaning the current node and parent meaning the parent of the current node.
         nn_->list(motions); // listing the motions in the nearest neighbor data structure. what is the list function? it is the function which lists the motions in the nearest neighbor data structure. -> indicates the member access operator,  what does it do? it is used to access the members of the class.
         for (auto &motion : motions)
         {
             if (motion->state != nullptr)
                 si_->freeState(motion->state);
             delete motion;
         }
     }
 }
  
ompl::base::PlannerStatus ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)
 {
     checkValidity();
     base::Goal *goal = pdef_->getGoal().get();
     auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
  
     while (const base::State *st = pis_.nextStart())
     {
         auto *motion = new Motion(si_);
         si_->copyState(motion->state, st); // copying the state st to the motion->state why? because we are storing the state in the motion->state.
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
  
     while (!ptc)
     {
         /* sample random state (with goal biasing) */
         if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
             goal_s->sampleGoal(rstate);
         else
             sampler_->sampleUniform(rstate);
  
         /* find closest state in the tree */
         Motion *nmotion = nn_->nearest(rmotion);
         base::State *dstate = rstate;
  
         /* find state to add */
         double d = si_->distance(nmotion->state, rstate);
         if (d > maxDistance_)
         {
             si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
             dstate = xstate;
         }
  
         if (si_->checkMotion(nmotion->state, dstate))
         {
             if (addIntermediateStates_)
             {
                 std::vector<base::State *> states;
                 const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);
  
                 if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                     si_->freeState(states[0]);
  
                 for (std::size_t i = 1; i < states.size(); ++i)
                 {
                     auto *motion = new Motion;
                     motion->state = states[i];
                     motion->parent = nmotion;
                     nn_->add(motion);
  
                     nmotion = motion;
                 }
             }
             else
             {
                 auto *motion = new Motion(si_);
                 si_->copyState(motion->state, dstate);
                 motion->parent = nmotion;
                 nn_->add(motion);
  
                 nmotion = motion;
             }
  
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
  
 void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
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