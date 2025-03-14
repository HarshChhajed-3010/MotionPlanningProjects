#include <vector>
#include <cmath>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <fstream>
#include <iostream>
#include <algorithm>


struct Segment {
    double x1, y1, x2, y2;
};


using Environment = std::vector<Segment>;


bool isStateInCollision(const ompl::base::State *state, const Environment &env) {
    const auto *compoundState = state->as<ompl::base::CompoundState>();
    const auto *pos = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = pos->values[0], y = pos->values[1];

    for (const auto &seg : env) {
        if (x >= std::min(seg.x1, seg.x2) && x <= std::max(seg.x1, seg.x2) &&
            y >= std::min(seg.y1, seg.y2) && y <= std::max(seg.y1, seg.y2)) {
            return true;  // In collision
        }
    }
    return false;  // Free space
}

bool isPointOutsideSquare(double x, double y, double rotation, double size, double centerX, double centerY) {
    // Rotate the point by -rotation
    double x_rot = (x - centerX) * cos(-rotation) - (y - centerY) * sin(-rotation) + centerX;
    double y_rot = (x - centerX) * sin(-rotation) + (y - centerY) * cos(-rotation) + centerY;

    // Check if the point is outside the square
    return x_rot < centerX - size / 2 || x_rot > centerX + size / 2 || y_rot < centerY - size / 2 || y_rot > centerY + size / 2;
}

bool isStateValid(const ompl::base::State *state) {
    const auto *compoundState = state->as<ompl::base::CompoundState>();
    const auto *pos = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = pos->values[0], y = pos->values[1];

    // Square parameters
    double squareSize = 2 * std::sqrt(2);
    double squareCenterX = -3.0;
    double squareCenterY = -2.0;
    double squareRotation = M_PI / 4.0;

    return isPointOutsideSquare(x, y, squareRotation, squareSize, squareCenterX, squareCenterY);
}

bool isPointOutsideRectangle(double x, double y, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    // Check if the point is outside the rectangle
    return (x < x1 || x > x2 || y < y1 || y > y2) && (x < x3 || x > x4 || y < y3 || y > y4);
}




// Set up collision checking
void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env) {
    ss.setStateValidityChecker([&env](const ompl::base::State *state) -> bool {
        return !isStateInCollision(state, env); // Valid if NOT in collision
    });

    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
}

// Plan for Scenario 1 using RRT
void planScenario1(ompl::geometric::SimpleSetup &ss) {
    auto planner = std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation());
    planner->setRange(0.1);
    ss.setPlanner(planner);
    ss.setup();

    if (ss.solve()) {
        std::ofstream pathFile("path1.txt");
        ss.getSolutionPath().printAsMatrix(pathFile);
        pathFile.close();
        std::cout << "Solution saved to path1.txt\n";
    } else {
        std::cout << "No solution found.\n";
    }
}

// Benchmark Scenario 1
void benchScenario1(ompl::geometric::SimpleSetup &ss) {
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");
    auto si = ss.getSpaceInformation();
    
    auto prm_star = std::make_shared<ompl::geometric::PRMstar>(si);
    prm_star->setProblemDefinition(ss.getProblemDefinition());
    b.addPlanner(prm_star);

    ompl::tools::Benchmark::Request request(20.0, 1024.0, 10, 0.5); 
    b.benchmark(request);
    b.saveResultsToFile("benchmark1.log");
    std::cout << "Benchmarking complete. Results saved to benchmark1.log.\n";
}

// Plan for Scenario 2 using RRT
void planScenario2(ompl::geometric::SimpleSetup &ss) {
    auto planner = std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation());
    planner->setRange(0.5);
    ss.setPlanner(planner);

    if (ss.solve(20.0)) {
        std::ofstream pathFile("path2.txt");
        ss.getSolutionPath().printAsMatrix(pathFile);
        pathFile.close();
        std::cout << "Solution saved to path2.txt\n";
    } else {
        std::cout << "No solution found.\n";
    }
}

// Benchmark Scenario 2
void benchScenario2(ompl::geometric::SimpleSetup &ss) {
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");
    auto si = ss.getSpaceInformation();
    
    auto prm_star = std::make_shared<ompl::geometric::PRMstar>(si);
    prm_star->setProblemDefinition(ss.getProblemDefinition());
    b.addPlanner(prm_star);

    ompl::tools::Benchmark::Request request(20.0, 1024.0, 10, 0.5);
    b.benchmark(request);
    b.saveResultsToFile("benchmark2.log");
    std::cout << "Benchmarking complete. Results saved to benchmark2.log.\n";
}

// Function to create the state space for the chain box
std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace() {
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    // Add a 2D position space (x, y)
    auto positionSpace = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    ompl::base::RealVectorBounds positionBounds(2);
    positionBounds.setLow(-5);
    positionBounds.setHigh(5);
    positionSpace->setBounds(positionBounds);
    space->addSubspace(positionSpace, 1.0);

    // Add a 1D orientation space (theta)
    auto orientationSpace = std::make_shared<ompl::base::SO2StateSpace>();
    space->addSubspace(orientationSpace, 1.0);

    // Add additional dimensions if needed (e.g., for a robotic arm with multiple joints)
    for (int i = 0; i < 4; ++i) {
        auto jointSpace = std::make_shared<ompl::base::RealVectorStateSpace>(1);
        ompl::base::RealVectorBounds jointBounds(1);
        jointBounds.setLow(-M_PI);
        jointBounds.setHigh(M_PI);
        jointSpace->setBounds(jointBounds);
        space->addSubspace(jointSpace, 1.0);
    }

    // Lock the state space
    space->lock();

    return space;
}

// Function to set up Scenario 1
void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal) {
    start = {-3, -3, 0, 0, 0, 0, 0};
    goal = {2, 2, 0, 0, 0, 0, -0.5 * M_PI};

    // Obstacle 1 - making a rectangle, with 4 lines, (2,-1) (2.8,-1) (2.8,0.5) (2,0.5)
    env.emplace_back(Segment{2, -1, 2.8, -1});
    env.emplace_back(Segment{2.8, -1, 2.8, 0.5});
    env.emplace_back(Segment{2.8, 0.5, 2, 0.5});
    env.emplace_back(Segment{2, 0.5, 2, -1});

    // Obstacle 2 - making a rectangle, with 4 lines, (3.2,-1) (4,-1) (4,0.5) (3.2,0.5)
    env.emplace_back(Segment{3.2, -1, 4, -1});
    env.emplace_back(Segment{4, -1, 4, 0.5});
    env.emplace_back(Segment{4, 0.5, 3.2, 0.5});
    env.emplace_back(Segment{3.2, 0.5, 3.2, -1});
}

// Function to set up Scenario 2
void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal) {
    start = {-4, -4, 0, 0, 0, 0, 0};
    goal = {3, 3, 0, 0, 0, 0, 0};

    // Obstacle 1 - making a rectangle, with 4 lines, (-1,-1) (1,-1) (1,1) (-1,1)
    env.emplace_back(Segment{-1, -1, 1, -1});
    env.emplace_back(Segment{1, -1, 1, 1});
    env.emplace_back(Segment{1, 1, -1, 1});
    env.emplace_back(Segment{-1, 1, -1, -1});
}

// Main Function
int main() {
    int scenario;
    Environment env;
    std::vector<double> startVec, goalVec;

    do {
        std::cout << "Plan for:\n (1) Robot Reaching Task\n (2) Robot Avoiding Task\n";
        std::cin >> scenario;
    } while (scenario < 1 || scenario > 2);

    if (scenario == 1) makeScenario1(env, startVec, goalVec);
    else makeScenario2(env, startVec, goalVec);

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);
    setupCollisionChecker(ss, env);

    ompl::base::ScopedState<> start(space), goal(space);
    for (size_t i = 0; i < startVec.size(); ++i) {
        start[i] = startVec[i];
        goal[i] = goalVec[i];
    }
    ss.setStartAndGoalStates(start, goal);

    if (scenario == 1) { planScenario1(ss); benchScenario1(ss); }
    else { planScenario2(ss); benchScenario2(ss); }

    return 0;
}