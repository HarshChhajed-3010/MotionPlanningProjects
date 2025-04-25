///////////////////////////////////////
// RBE 550 - Motion Planning
// Project 4: RG-RRT Implementation
// Authors: Pranay Katyal, Harsh Chhajed
//////////////////////////////////////


#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <fstream>  // for file I/O

// Include the necessary OMPL headers for state and control spaces.
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "CollisionChecking.h"
#include "RG-RRT.h"  //RG-RRT implementation

#include <Eigen/Core>

namespace ob = ompl::base;
namespace oc = ompl::control;

//---------------------------------------------------------
// Projection for the Car: Projects the compound state space (SE2 + velocity)
// to a 2D projection (x, y)
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Extract x and y from the SE2 component of the compound state
        const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
        const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);
        
        projection[0] = se2State->getX();
        projection[1] = se2State->getY();
    }
};

//---------------------------------------------------------
// Post-propagation function to ensure valid orientation angles
void postPropagate(const ob::State* /*state*/, const oc::Control* /*control*/, 
                   const double /*duration*/, ob::State* result)
{
    // Ensuring that the car's resulting orientation lies between 0 and 2*pi
    ob::CompoundStateSpace::StateType* compoundState = result->as<ob::CompoundStateSpace::StateType>();
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(compoundState->as<ob::SE2StateSpace::StateType>(0)->as<ob::SO2StateSpace::StateType>(1));
}

//---------------------------------------------------------
// ODE for the Car dynamics.
// Compound state with SE2 (x, y, theta) + velocity
// Control is u = [omega, a] (steering rate, acceleration)
void carODE(const oc::ODESolver::StateType &q, const oc::Control *control,
            oc::ODESolver::StateType &qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double omega = u[0];  // steering rate
    double a = u[1];      // acceleration
    
    double theta = q[2];  // orientation
    double v = q[3];      // velocity

    qdot.resize(q.size(), 0);
    qdot[0] = v * std::cos(theta);  // x_dot
    qdot[1] = v * std::sin(theta);  // y_dot
    qdot[2] = omega;                // theta_dot
    qdot[3] = a;                    // v_dot
}

//---------------------------------------------------------
// Create a street environment by filling the obstacles vector.
void makeStreet(std::vector<Obstacle> &obstacles)
{
    // define the obstacles in the environment
    obstacles.push_back({-7.0, -10.0, 1.0, 20.0});
    obstacles.push_back({6.0, -10.0, 1.0, 20.0});
    obstacles.push_back({-5.0, 2.0, 3.0, 2.0});
    obstacles.push_back({2.0, -3.0, 3.0, 2.0});
    obstacles.push_back({-1.5, 0.0, 3.0, 1.0});
    
    // Write obstacles to file for visualization
    std::ofstream outFile("obstacles.txt");
    if (outFile.is_open()) {
        for (const auto& obs : obstacles) {
            outFile << obs.x << " " << obs.y << " " << obs.width << " " << obs.height << std::endl;
        }
        outFile.close();
    }
}

//---------------------------------------------------------
// Debugging function to visualize state propagation
void debugVisualizePropagation(const ob::SpaceInformationPtr& si)
{
    // Create a random valid state
    ob::State* start = si->allocState();
    auto* se2start = start->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    se2start->setX(-5.0);
    se2start->setY(-6.0);
    se2start->setYaw(M_PI/2); // facing up
    // Set initial velocity
    start->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;
    
    // Create a random control
    auto* spaceInfo = dynamic_cast<oc::SpaceInformation*>(si.get());
    auto* control = spaceInfo->allocControl();
    auto* controlVector = control->as<oc::RealVectorControlSpace::ControlType>();
    controlVector->values[0] = 0.1; // small steering
    controlVector->values[1] = 0.5; // acceleration
    
    // Propagate and visualize
    ob::State* result = si->allocState();
    std::vector<ob::State*> states;
    
    std::cout << "Debug Propagation:" << std::endl;
    std::cout << "Starting at: ";
    si->printState(start, std::cout);
    std::cout << std::endl;
    
    // Propagate for 10 steps
    for (int i = 0; i < 10; i++) {
        spaceInfo->propagate(start, control, 0.1, result);
        std::cout << "Step " << i << ": ";
        si->printState(result, std::cout);
        std::cout << std::endl;
        
        // Copy result to start for next iteration
        si->copyState(start, result);
    }
    
    // Clean up
    si->freeState(start);
    si->freeState(result);
    spaceInfo->freeControl(control);
}

//---------------------------------------------------------
// Validity checker for the car state
bool isCarStateValid(const ob::State *state, const std::vector<Obstacle> &obstacles)
{
    const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
    const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);
    const auto *velocityState = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
    
    double x = se2State->getX();
    double y = se2State->getY();
    double theta = se2State->getYaw();
    double v = velocityState->values[0];
    
    // Check if velocity is within bounds (-5.0, 5.0)
    if (v < -5.0 || v > 5.0) {
        return false;
    }
    
    // Check if the state is within workspace bounds (-6, 6) x (-6, 6)
    if (x < -6.0 || x > 6.0 || y < -6.0 || y > 6.0) {
        return false;
    }
    
    // Car dimensions
    double carLength = 0.8;  
    double carWidth = 0.4;   
    
    // Create a Car object for collision checking
    Car car = {
        x,           // x position
        y,           // y position
        carLength,   // length
        carWidth,    // width
        theta        // orientation
    };
    
    // Check collision with obstacles
    for (const auto& obs : obstacles) {
        if (checkCollision(car, obs)) {
            return false;
        }
    }
    
    return true;
}

//---------------------------------------------------------
// Create and setup the car's state space, control space, and everything needed for planning
oc::SimpleSetupPtr createCar(const std::vector<Obstacle> &obstacles)
{
    // Create a compound state space
    auto space = std::make_shared<ob::CompoundStateSpace>();
    
    // SE2 component for position and orientation
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds se2Bounds(2);
    se2Bounds.setLow(-6.0);
    se2Bounds.setHigh(6.0);
    se2->setBounds(se2Bounds);
    
    // RealVector component for velocity
    auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds velocityBounds(1);
    velocityBounds.setLow(-5.0);
    velocityBounds.setHigh(5.0);
    velocitySpace->setBounds(velocityBounds);
    
    // Add the components to the compound space
    space->addSubspace(se2, 1.0);
    space->addSubspace(velocitySpace, 0.5);
    
    // Create the control space - angle rate and acceleration
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -M_PI/3);    // steering rate lower bound
    controlBounds.setHigh(0, M_PI/3);    // steering rate upper bound
    controlBounds.setLow(1, -2.0);       // acceleration lower bound
    controlBounds.setHigh(1, 2.0);       // acceleration upper bound
    controlSpace->setBounds(controlBounds);
    
    // Create the SimpleSetup object
    auto ss = std::make_shared<oc::SimpleSetup>(controlSpace);
    
    // Set up ODE solver with post-propagation step
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));
    
    // Set the state validity checker
    ss->setStateValidityChecker(
        [&obstacles](const ob::State *state) -> bool {
            return isCarStateValid(state, obstacles);
        });
    
    // Register the projection for KPIECE1
    space->registerProjection("CarProjection", std::make_shared<CarProjection>(space.get()));
    
    return ss;
}

//---------------------------------------------------------
// Perform motion planning for the car using the chosen planner
void planCar(oc::SimpleSetupPtr &ss, int choice)
{
    
    // Define start state
    ob::ScopedState<ob::CompoundStateSpace> start(ss->getStateSpace());
    auto se2 = start->as<ob::SE2StateSpace::StateType>(0);
    se2->setX(-5.0);
    se2->setY(-6.0);
    se2->setYaw(M_PI/2);  // facing up
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;  // initial velocity
    
    // Define goal state
    ob::ScopedState<ob::CompoundStateSpace> goal(ss->getStateSpace());
    auto se2Goal = goal->as<ob::SE2StateSpace::StateType>(0);
    se2Goal->setX(-4.0);
    se2Goal->setY(5.0);
    se2Goal->setYaw(M_PI/2);  // facing up
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;  // goal velocity
    
    // Narrower goal threshold
    ss->setStartAndGoalStates(start, goal, 0.3);
    
    // Verify state validity
    std::cout << "Start state valid: " << (ss->getSpaceInformation()->isValid(start.get()) ? "Yes" : "No") << std::endl;
    std::cout << "Goal state valid: " << (ss->getSpaceInformation()->isValid(goal.get()) ? "Yes" : "No") << std::endl;
    
    // Set propagation parameters - smaller step size for better accuracy
    ss->getSpaceInformation()->setMinMaxControlDuration(5, 30);  // increased duration
    ss->getSpaceInformation()->setPropagationStepSize(0.05);  // Smaller step size
    
    // Choose planner based on user input
    ob::PlannerPtr planner;
    
    if (choice == 1)
    {
        // RRT planner with tuned parameters
        auto rrt = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        rrt->setGoalBias(0.15);  // Increased goal bias
        rrt->setIntermediateStates(true);  // Include intermediate states
        ss->setPlanner(rrt);
    }
    else if (choice == 2)
    {
        // KPIECE1 planner with tuned parameters
        auto kpiece = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        kpiece->setProjectionEvaluator("CarProjection");
        kpiece->setGoalBias(0.1);
        kpiece->setBorderFraction(0.8);
        ss->setPlanner(kpiece);
    }
    else if (choice == 3)
    {
        // RG-RRT planner
        auto rgrrt = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        
        ss->setPlanner(rgrrt);
    }
    else
    {
        std::cerr << "Invalid planner choice. Defaulting to RRT." << std::endl;
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    
    // Setup and solve
    ss->setup();
    std::cout << "Planning started..." << std::endl;
    ob::PlannerStatus solved = ss->solve(60.0); // increase the time here if it doesnt solve in 60 
    
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        
        // Get the solution path and interpolate it
        oc::PathControl &path = ss->getSolutionPath();
        std::cout << "Original path has " << path.getStateCount() << " states" << std::endl;
        
        // Use default interpolation
        path.interpolate();
        std::cout << "After interpolation: " << path.getStateCount() << " states" << std::endl;
        
  
        // Write the solution path to file
        std::ofstream outFile("car_path.txt");
        if (outFile.is_open())
        {
            outFile << "SE2 " << "0.4" << std::endl;  // Header with car width
            path.asGeometric().printAsMatrix(outFile);
            outFile.close();
            std::cout << "Solution path written to car_path.txt" << std::endl;
        }
        else
        {
            std::cerr << "Error: Unable to open car_path.txt for writing." << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found for the car problem." << std::endl;
    }
}

//---------------------------------------------------------
// Benchmarking function for the car
void benchmarkCar(oc::SimpleSetupPtr &ss)
{
    // Define start state
    ob::ScopedState<ob::CompoundStateSpace> start(ss->getStateSpace());
    auto se2 = start->as<ob::SE2StateSpace::StateType>(0);
    se2->setX(-5.0);
    se2->setY(-6.0);
    se2->setYaw(M_PI/2);  // facing up
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;  // initial velocity

    // Define goal state
    ob::ScopedState<ob::CompoundStateSpace> goal(ss->getStateSpace());
    auto se2Goal = goal->as<ob::SE2StateSpace::StateType>(0);
    se2Goal->setX(-4.0);
    se2Goal->setY(5.0);
    se2Goal->setYaw(M_PI/2);  // facing up
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;  // goal velocity

    // Set start and goal with the same threshold as planning
    ss->setStartAndGoalStates(start, goal, 0.3);

    // Verify state validity (optional for benchmark)
    std::cout << "Start state valid: " << (ss->getSpaceInformation()->isValid(start.get()) ? "Yes" : "No") << std::endl;
    std::cout << "Goal state valid: " << (ss->getSpaceInformation()->isValid(goal.get()) ? "Yes" : "No") << std::endl;

    // Setup the problem (important for planners to initialize correctly)
    ss->setup();

    // Create the benchmark object
    ompl::tools::Benchmark b(*ss.get(), "Car_Motion_Planning");
    
    // Configure standard RRT planner
    auto rrt = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    rrt->setProblemDefinition(ss->getProblemDefinition());
    rrt->setName("RRT_Vanilla");
    rrt->setGoalBias(0.15);
    b.addPlanner(rrt);

    // Configure KPIECE1 planner
    auto kpiece = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    kpiece->setProblemDefinition(ss->getProblemDefinition());
    kpiece->setName("KPIECE1");
    kpiece->setProjectionEvaluator("CarProjection");
    kpiece->setGoalBias(0.1);  
    kpiece->setBorderFraction(0.8);
    b.addPlanner(kpiece);
    
    // Configure RG-RRT planner
    auto rgrrt = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
    rgrrt->setProblemDefinition(ss->getProblemDefinition());
    rgrrt->setName("RG-RRT");
    b.addPlanner(rgrrt);
    
    // Configure and run benchmark
    ompl::tools::Benchmark::Request request(60.0, 2048.0, 10, 0.5); // increased time and memory
    b.benchmark(request);
    b.saveResultsToFile("car_benchmark_results.log");
    
    std::cout << "Benchmark complete. Results saved to car_benchmark_results.log" << std::endl;
}

//---------------------------------------------------------
// Main function
int main(int /* argc */, char ** /* argv */)
{
    std::vector<Obstacle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    oc::SimpleSetupPtr ss = createCar(obstacles);

    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "Select Planner:" << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);
        planCar(ss, planner);
    }
    else if (choice == 2)
    {
        benchmarkCar(ss);
    }
    else
    {
        std::cerr << "Invalid choice." << std::endl;
    }

    return 0;
}