///////////////////////////////////////
// RBE 550 - Motion Planning
// Project 4: RG-RRT Implementation
// Authors: Pranay Katyal, Harsh Chhajed
//////////////////////////////////////
#include <iostream>
#include <cmath>
#include <fstream>
#include <memory>

// Include the necessary OMPL headers.
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>      // Added
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>   // Added
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h> // Added
#include "RG-RRT.h"  
#include <ompl/tools/benchmark/Benchmark.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

//---------------------------------------------------------
// Projection for the Pendulum: reduce 2D state [theta, omega] to 1D (theta)
class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 1;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // The state is a RealVectorStateSpace state of dimension 2.
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection[0] = vals[0]; // theta
    }
};

//---------------------------------------------------------
// ODE for the Pendulum dynamics.
// q = [theta, omega] and control u = [tau]
// where:
//   theta_dot = omega
//   omega_dot = -g*cos(theta) + tau
void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *control,
                 oc::ODESolver::StateType &qdot)
{
    const double g = 9.81;
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double tau = u[0];

    qdot.resize(q.size(), 0);
    qdot[0] = q[1];                     // theta_dot = omega
    qdot[1] = -g * std::cos(q[0]) + tau;  // omega_dot = -g*cos(theta) + tau
}

//---------------------------------------------------------
// Validity checker for the pendulum state.
// Here we ensure that |omega| is <= 10.
bool isPendulumStateValid(const ob::State *state)
{
    const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    double omega = vals[1];
    return (std::fabs(omega) <= 10.0);
}

//---------------------------------------------------------
// Create and setup the pendulum's state space, control space, state propagator,
// and state validity checker.
oc::SimpleSetupPtr createPendulum(double torqueBound)
{
    // State space: 2-dimensional [theta, omega]
    ob::StateSpacePtr stateSpace(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);
    bounds.setHigh(0, M_PI);
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Control space: 1-dimensional for torque.
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(stateSpace, 1));
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torqueBound);
    cbounds.setHigh(torqueBound);
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(controlSpace));

    // Use ODEBasicSolver with pendulumODE.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(ss->getSpaceInformation(), pendulumODE));
    ss->getSpaceInformation()->setStatePropagator(
        oc::ODESolver::getStatePropagator(odeSolver, nullptr)
    );

    ss->setStateValidityChecker(isPendulumStateValid);

    // Register the projection evaluator.
    stateSpace->registerProjection("PendulumProjection", std::make_shared<PendulumProjection>(stateSpace.get()));

    return ss;
}

//---------------------------------------------------------
// Perform motion planning for the pendulum using the chosen planner.
void planPendulum(oc::SimpleSetupPtr &ss, int choice)
{
    // Define start state: (-pi/2, 0)
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2;
    start[1] = 0.0;

    // Define goal state: (pi/2, 0)
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = M_PI / 2;
    goal[1] = 0.0;

    ss->setStartAndGoalStates(start, goal, 0.5);

    ss->getSpaceInformation()->setMinMaxControlDuration(1, 10);
    ss->getSpaceInformation()->setPropagationStepSize(0.01);

    if (choice == 1)
    {
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }
    else if (choice == 2)
    {
        auto planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        planner->setGoalBias(0.01); // Bias toward goal
// at 0.1, Torque = 3,  worked nicely, 0.01 is too slow and it seems to require more effort.
// at 0.01 Torque = 5,  worked nicely, 0.1 is seems to overshoot and thus require more effort.
// at 0.1, 0.01 Torque = 10,  overshot and required more effort, but at 0.001 it worked nicely.
// at 0.1, Torque = 3,  worked nicely, 0.01 is too slow and it seems to require more effort.
// at 0.01 Torque = 5,  worked nicely, 0.1 is seems to overshoot and thus require more effort.
       
        ss->setPlanner(planner);
    }
    else if (choice == 3)
    {
        // Ensure RGRRT is correctly instantiated
        std::cerr << "RGRRT executing." << std::endl;
        auto planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        planner->setGoalBias(0.05); // Set goal bias for RGRRT
        planner->setIntermediateStates(true); // Enable intermediate states
        ss->setPlanner(planner);
    }
    else
    {
        std::cerr << "Invalid planner choice. Defaulting to RRT." << std::endl;
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }

    ob::PlannerStatus solved = ss->solve(30.0);
    if (solved)
    {
        std::cout << "Pendulum plan found:" << std::endl;
        // Write solution path to "pendulum_path.txt"
        std::ofstream outFile("pendulum_path.txt");
        if (outFile.is_open())
        {
            ss->getSolutionPath().printAsMatrix(outFile);
            outFile.close();
            std::cout << "Solution path written to pendulum_path.txt" << std::endl;
        }
        else
        {
            std::cerr << "Error: Unable to open pendulum_path.txt for writing." << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found for the pendulum problem." << std::endl;
    }
}

//---------------------------------------------------------
// Benchmarking function for the pendulum
void benchmarkPendulum(oc::SimpleSetupPtr &ss)
{
    // Define start state: (-pi/2, 0)
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2;
    start[1] = 0.0;

    // Define goal state: (pi/2, 0)
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = M_PI / 2;
    goal[1] = 0.0;

    // Set start and goal states
    ss->setStartAndGoalStates(start, goal, 0.5);

    // Setup the problem
    ss->setup();

    // Create the benchmark object
    ompl::tools::Benchmark b(*ss.get(), "Pendulum Swing-up Benchmark");

    // Configure RRT planner
    auto rrt = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    rrt->setName("RRT");
    rrt->setGoalBias(0.1);
    b.addPlanner(rrt);

    // Configure KPIECE1 planner
    auto kpiece = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    kpiece->setName("KPIECE1");
    kpiece->setProjectionEvaluator("PendulumProjection");
    kpiece->setGoalBias(0.01);
    kpiece->setBorderFraction(0.8);
    b.addPlanner(kpiece);

    // Configure RG-RRT planner
    auto rgrrt = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
    rgrrt->setName("RG-RRT");
    rgrrt->setGoalBias(0.05);
    b.addPlanner(rgrrt);

    // Configure benchmark parameters
    ompl::tools::Benchmark::Request request;
    request.maxTime = 30.0;       // Maximum time per run
    request.maxMem = 2048.0;      // Maximum memory allowed
    request.runCount = 50;        // Number of runs per planner
    request.displayProgress = true;

    // Run benchmark
    b.benchmark(request);
    
    // Save results to file
    std::ostringstream filename;
    filename << "pendulum_benchmark_torque" << ss->getControlSpace()->as<oc::RealVectorControlSpace>()->getBounds().high[0] << ".log";
    b.saveResultsToFile(filename.str().c_str());

    std::cout << "Benchmark complete. Results saved to " << filename.str() << std::endl;
}

//---------------------------------------------------------
//---------------------------------------------------------
// Main function for the pendulum planning executable.
int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    // Torque selection is needed for both planning and benchmarking
    double torques[] = {3.0, 5.0, 10.0};
    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1) 3" << std::endl;
        std::cout << " (2) 5" << std::endl;
        std::cout << " (3) 10" << std::endl;
        std::cin >> which;
    } while (which < 1 || which > 3);
    double torque = torques[which - 1];

    oc::SimpleSetupPtr ss = createPendulum(torque);

    if (choice == 1)
    {
        // Planner selection only for planning
        int planner;
        do
        {
            std::cout << "Select Planner:" << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);
        
        planPendulum(ss, planner);
    }
    else  // Benchmarking
    {
        benchmarkPendulum(ss);
    }

    return 0;
}