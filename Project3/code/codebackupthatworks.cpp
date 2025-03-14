#include "KinematicChain.h"
#include <cmath>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/base/Cost.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <fstream>
#include <iostream>
#include <algorithm>

struct Point {
    double x, y;
    Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};

struct Line {
    double a, b, c;
    Line(const Point& p1, const Point& p2) {
        a = p2.y - p1.y;
        b = p1.x - p2.x;
        c = a * p1.x + b * p1.y;
    }
};

struct Rectangle {
    double x, y, width, height;
};

// Helper function declarations
Point rotatePoint(const Point& p, double angle);
std::vector<Point> getSquareCorners(const Point& center, double l, double angle);
std::vector<Point> getRectangleCorners(const Rectangle& obstacle);
std::vector<Point> getSegmentCorners(const Segment& segment);
bool isPointOnSegment(const Point& p, const Point& a, const Point& b);
bool getIntersection(const Point& A, const Point& B, const Point& C, const Point& D, Point& intersection);
bool doSegmentsIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2);

void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal) {
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2; 
    goal[1] = 2; 
    goal[2] = 0; 
    goal[4] = -0.5 * M_PI;

    // Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    // Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);
}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal) {
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 

    // Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss) {
    auto planner = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
    planner->setRange(0.1);
    ss.setPlanner(planner);
    ss.setup();

    if (ss.solve(15.0)) {
        std::ofstream pathFile("narrow.txt");
        ss.getSolutionPath().printAsMatrix(pathFile);
        pathFile.close();
        std::cout << "Solution saved to narrow.txt\n";
    } else {
        std::cout << "No solution found.\n";
    }
}

void benchScenario1(ompl::geometric::SimpleSetup &ss) {
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");
    // RRT Connect
    auto rrt_connect = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
    rrt_connect->setProblemDefinition(ss.getProblemDefinition());
    rrt_connect->setName("RRTConnect");
    b.addPlanner(rrt_connect);
    
    // Uniform Sampling
    auto prm_uniform = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    prm_uniform->setProblemDefinition(ss.getProblemDefinition());
    prm_uniform->setName("PRM_Uniform");
    b.addPlanner(prm_uniform);

    // Gaussian Sampling
    auto prm_gaussian = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    prm_gaussian->setProblemDefinition(ss.getProblemDefinition());
    prm_gaussian->setName("PRM_Gaussian");
    b.addPlanner(prm_gaussian);

    // Bridge Sampling
    auto prm_bridge = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    prm_bridge->setProblemDefinition(ss.getProblemDefinition());
    prm_bridge->setName("PRM_Bridge");
    b.addPlanner(prm_bridge);

    // Obstacle-based Sampling
    auto prm_obstacle = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    prm_obstacle->setProblemDefinition(ss.getProblemDefinition());
    prm_obstacle->setName("PRM_Obstacle");
    b.addPlanner(prm_obstacle);

    ompl::tools::Benchmark::Request request(20.0, 1024.0, 20, 0.5);
    b.benchmark(request);
    b.saveResultsToFile("benchmarkScenario1.log");
}

class CustomClearanceObjective : public ompl::base::OptimizationObjective {
    public:
        CustomClearanceObjective(const ompl::base::SpaceInformationPtr &si, const Environment &env, double robotHalfWidth, double obstacleHalfWidth)
            : ompl::base::OptimizationObjective(si), env_(env), robotHalfWidth_(robotHalfWidth), obstacleHalfWidth_(obstacleHalfWidth) {
            setCostThreshold(ompl::base::Cost(1e6)); // Upper bound on cost (lower bound for inverse)
        }
    
        ompl::base::Cost stateCost(const ompl::base::State *s) const override {
            const auto *compound = s->as<ompl::base::CompoundStateSpace::StateType>();
    
            // Get base position
            const auto *pos = compound->as<ompl::base::RealVectorStateSpace::StateType>(0);
            double x = pos->values[0], y = pos->values[1];
    
            // Obstacle is at the center of the environment (0, 0)
            double obstacleX = 0.0, obstacleY = 0.0;
    
            // Calculate the distance between the centers of the robot and the obstacle
            double dx = std::abs(x - obstacleX);
            double dy = std::abs(y - obstacleY);
    
            // Calculate the minimum distance between the edges of the robot and the obstacle
            double edgeDistanceX = dx - (robotHalfWidth_ + obstacleHalfWidth_);
            double edgeDistanceY = dy - (robotHalfWidth_ + obstacleHalfWidth_);
    
            // If either edge distance is negative, it means collision
            if (edgeDistanceX < 0.0 || edgeDistanceY < 0.0) {
                return ompl::base::Cost(std::numeric_limits<double>::infinity());
            }
    
            // The clearance is the minimum of the edge distances
            double clearance = std::min(edgeDistanceX, edgeDistanceY);
    
            // Return inverse of clearance as cost (lower cost = higher clearance)
            return ompl::base::Cost(1.0 / (clearance + 1e-6)); // +1e-6 to avoid division by zero
        }
    
        ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override {
            double cost1 = stateCost(s1).value();
            double cost2 = stateCost(s2).value();
    
            // Ensure valid costs
            if (std::isinf(cost1)) return ompl::base::Cost(cost1);
            if (std::isinf(cost2)) return ompl::base::Cost(cost2);
    
            // Sum costs to make them additive (instead of taking the minimum)
            return ompl::base::Cost(cost1 + cost2);
        }
    
        bool isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const override {
            return c1.value() < c2.value(); // Lower cost is better (equivalent to higher clearance)
        }
    
    private:
        const Environment &env_;
        double robotHalfWidth_; 
        double obstacleHalfWidth_; 
    };

void planScenario2(ompl::geometric::SimpleSetup &ss, const Environment &env) {
    std::cout << "Planning for scenario 2...\n";

    // Define robot and obstacle half-widths
    double robotHalfWidth = 0.5; // Half-width of the robot (square)
    double obstacleHalfWidth = 1.0; // Half-width of the obstacle (square)

    // Create custom clearance objective
    auto clearanceObj = std::make_shared<CustomClearanceObjective>(
        ss.getSpaceInformation(), env, robotHalfWidth, obstacleHalfWidth
    );
    clearanceObj->setCostThreshold(ompl::base::Cost(0.0)); // Lower bound on cost

    // Set the optimization objective
    ss.setOptimizationObjective(clearanceObj);

    std::cout << "Setting up planner...\n";

    // Use RRT* with tuned parameters
    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    planner->setRange(0.2);  // Increased range for better exploration
    planner->setGoalBias(0.05);  // Increased goal bias
    ss.setPlanner(planner);
    ss.setup();

    std::cout << "Planning...\n";

    // Set better termination conditions
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(120.0);

    std::cout << "Solving...\n";
    if (ss.solve(ptc)) {
        std::cout << "Solution found!\n";
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        std::cout << "Path length: " << path.length() << std::endl;

        // Interpolate the path for smoother output
        path.interpolate(50); // Add more states for smoother path
        std::cout << "Interpolated path length: " << path.length() << std::endl;

        // Save the path to a file
        std::ofstream pathFile("clear.txt");
        path.printAsMatrix(pathFile);
        pathFile.close();
        std::cout << "Solution saved to clear.txt\n";

        // Print the achieved minimum clearance
        std::cout << "Achieved minimum clearance: " 
                   << path.cost(clearanceObj).value() << std::endl;
    } else {
        std::cout << "No solution found within time limit.\n";
    }
}

void benchScenario2(ompl::geometric::SimpleSetup &ss, const Environment &env) {
    std::cout << "Benchmarking scenario 2...\n";

    // Define robot and obstacle half-widths
    double robotHalfWidth = 0.5;
    double obstacleHalfWidth = 1.0;

    // Create custom clearance objective
    auto clearanceObj = std::make_shared<CustomClearanceObjective>(
        ss.getSpaceInformation(), env, robotHalfWidth, obstacleHalfWidth
    );
    clearanceObj->setCostThreshold(ompl::base::Cost(0.0));

    // Set the optimization objective
    ss.setOptimizationObjective(clearanceObj);

    std::cout << "Setting up benchmark...\n";

    // Create benchmark object
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    // Ensure space is set up
    ss.getSpaceInformation()->setup();

    try {
        // Add planners to benchmark
        auto rrt_star = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
        rrt_star->setName("RRT*");
        rrt_star->setProblemDefinition(ss.getProblemDefinition());
        rrt_star->setup();
        b.addPlanner(rrt_star);

        auto prm_star = std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation());
        prm_star->setName("PRM*");
        prm_star->setProblemDefinition(ss.getProblemDefinition());
        prm_star->setup();
        b.addPlanner(prm_star);

        auto rrt_sharp = std::make_shared<ompl::geometric::RRTsharp>(ss.getSpaceInformation());
        rrt_sharp->setName("RRT#");
        rrt_sharp->setProblemDefinition(ss.getProblemDefinition());
        rrt_sharp->setup();
        b.addPlanner(rrt_sharp);

        // Refined benchmark request
        ompl::tools::Benchmark::Request request(
            60.0,    // time limit (seconds)
            4096.0,  // memory limit (MB)
            10,      // run count
            0.5      // display progress
        );

        std::cout << "Running benchmark...\n";
        b.benchmark(request);

        // Save results
        std::cout << "Saving results...\n";
        b.saveResultsToFile("benchmarkScenario2.log");
        std::cout << "Benchmarking complete.\n";
    }
    catch (const ompl::Exception& e) {
        std::cerr << "OMPL Exception during benchmarking: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Standard Exception during benchmarking: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown exception during benchmarking." << std::endl;
    }
}

std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace() {
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    // Box center position (x, y) ∈ [-5, 5] x [-5, 5]
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    r2->setBounds(-5, 5);
    space->addSubspace(r2, 1.0);  

    // Box orientation θ ∈ SO(2)
    auto so2_box = std::make_shared<ompl::base::SO2StateSpace>();
    space->addSubspace(so2_box, 1.0);  

    // 4 revolute joints in the chain (each ∈ SO(2))
    for (int i = 0; i < 4; i++) {
        auto so2_joint = std::make_shared<ompl::base::SO2StateSpace>();
        space->addSubspace(so2_joint, 1.0);
    }

    return space;
}

// Helper functions
Point rotatePoint(const Point& p, double angle) {
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);
    return Point(
        p.x * cosAngle - p.y * sinAngle,
        p.x * sinAngle + p.y * cosAngle
    );
}

std::vector<Point> getSquareCorners(const Point& center, double l, double angle) {
    double s = l / 2.0;

    std::vector<Point> corners = {
        Point(-s, -s),
        Point(s, -s),
        Point(s, s),
        Point(-s, s)
    };

    for (auto& corner : corners) {
        corner = rotatePoint(corner, angle);
        corner.x += center.x;
        corner.y += center.y;
    }

    return corners;
}

std::vector<Point> getRectangleCorners(const Rectangle& obstacle) {
    return {
        Point(obstacle.x, obstacle.y),
        Point(obstacle.x + obstacle.width, obstacle.y),
        Point(obstacle.x + obstacle.width, obstacle.y + obstacle.height),
        Point(obstacle.x, obstacle.y + obstacle.height)
    };
}

std::vector<Point> getSegmentCorners(const Segment& segment) {
    return {
        Point(segment.x0, segment.y0),
        Point(segment.x1, segment.y0),
        Point(segment.x1, segment.y1),
        Point(segment.x0, segment.y1)
    };
}

bool isPointOnSegment(const Point& p, const Point& a, const Point& b) {
    return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
           std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

bool getIntersection(const Point& A, const Point& B, const Point& C, const Point& D, Point& intersection) {
    Line line1(A, B);
    Line line2(C, D);

    double det = line1.a * line2.b - line2.a * line1.b;

    if (std::abs(det) < 1e-9) return false;

    intersection.x = (line2.b * line1.c - line1.b * line2.c) / det;
    intersection.y = (line1.a * line2.c - line2.a * line1.c) / det;

    return isPointOnSegment(intersection, A, B) && isPointOnSegment(intersection, C, D);
}

bool isSegmentIntersectingBase(const Point& p1, const Point& q1, double x, double y, double theta, double baseSize) {
    // Get the corners of the base square
    std::vector<Point> baseCorners = getSquareCorners(Point(x, y), baseSize, theta);
    
    // Check intersection with each edge of the base square
    for (int i = 0; i < 4; i++) {
        Point baseP = baseCorners[i];
        Point baseQ = baseCorners[(i + 1) % 4];
        
        if (doSegmentsIntersect(p1, q1, baseP, baseQ)) {
            return true;
        }
    }
    
    return false;
}

bool doSegmentsIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
    auto orientation = [](const Point& a, const Point& b, const Point& c) {
        double val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
        if (val > 0) return 1;  // Clockwise
        if (val < 0) return -1; // Counterclockwise
        return 0;               // Collinear
    };

    auto onSegment = [](const Point& p, const Point& q, const Point& r) {
        return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x)) &&
                (q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
    };

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true; // General intersection case

    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}

bool isSquareIntersectingObstacles(const std::vector<Point>& squareCorners, const std::vector<Point>& obstacleCorners) {
    for (int i = 0; i < 4; ++i) {
        Point A = squareCorners[i];
        Point B = squareCorners[(i + 1) % 4];

        for (int j = 0; j < 4; ++j) {
            Point C = obstacleCorners[j];
            Point D = obstacleCorners[(j + 1) % 4];

            if (doSegmentsIntersect(A, B, C, D)) {
                return true;
            }
        }
    }
    return false;
}

bool isValidSquare(double x, double y, double theta, double sideLength, 
    const Environment& obstacles, double hbound, double lbound) {
    Point center(x, y);
    std::vector<Point> squareCorners = getSquareCorners(center, sideLength, theta);

    // Check if square is within bounds
    for (const auto& corner : squareCorners) {
        if (corner.x > lbound || corner.y > hbound || corner.x < -lbound || corner.y < -hbound) {
            return false;
        }
    }

    // Check for collisions with obstacles
    for (const auto& obstacle : obstacles) {
        std::vector<Point> obstacleCorners = getSegmentCorners(obstacle);
        if (isSquareIntersectingObstacles(squareCorners, obstacleCorners)) {
            return false;
        }
    }

    return true;
}

Point pairToPoint(const std::pair<double, double>& p) {
    return Point(p.first, p.second);
}

std::pair<double, double> pointToPair(const Point& p) {
    return {p.x, p.y};
}

void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env, double baseSize = 1.0) {
    ss.setStateValidityChecker([&env, baseSize](const ompl::base::State *state) {
        const auto *compound = state->as<ompl::base::CompoundStateSpace::StateType>();

        // Extract base (x, y, θ)
        const auto *pos = compound->as<ompl::base::RealVectorStateSpace::StateType>(0);
        double x = pos->values[0], y = pos->values[1];
        double theta = compound->as<ompl::base::SO2StateSpace::StateType>(1)->value;

        // Check if base position is valid (check with boundary and obstacles)
        if (!isValidSquare(x, y, theta, baseSize, env, 5.0, 5.0)) {
            return false;
        }

        // Get chain joint angles
        std::vector<double> jointAngles(4);
        for (int i = 0; i < 4; i++)
            jointAngles[i] = compound->as<ompl::base::SO2StateSpace::StateType>(2 + i)->value;

        // Compute chain link positions
        std::vector<Point> link_positions;
        std::vector<std::pair<Point, Point>> link_segments;
        double link_x = x, link_y = y, angle = theta;
        double link_length = 1.0; // Assume each link has a length of 1

        // First joint position (skip checking this segment since it's in the base)
        angle += jointAngles[0];
        double next_x = link_x + link_length * cos(angle);
        double next_y = link_y + link_length * sin(angle);
        link_segments.push_back({Point(link_x, link_y), Point(next_x, next_y)});
        link_positions.emplace_back(next_x, next_y);
        link_x = next_x;
        link_y = next_y;

        for (size_t i = 1; i < jointAngles.size(); i++) {
            angle += jointAngles[i];  // Update angle
            double next_x = link_x + link_length * cos(angle);
            double next_y = link_y + link_length * sin(angle);
            
            // Check if current link intersects with base
            if (isSegmentIntersectingBase(Point(link_x, link_y), Point(next_x, next_y), 
                                        x, y, theta, baseSize)) {
                return false;
            }

            link_segments.push_back({Point(link_x, link_y), Point(next_x, next_y)});
            link_positions.emplace_back(next_x, next_y);

            link_x = next_x;
            link_y = next_y;
        }

        // Check for self-intersection
        for (size_t i = 0; i < link_segments.size(); ++i) {
            for (size_t j = i + 2; j < link_segments.size(); ++j) {  // Ensure non-adjacent links
                if (doSegmentsIntersect(link_segments[i].first, link_segments[i].second,
                                        link_segments[j].first, link_segments[j].second))
                    return false;
            }
        }

        // Check for collisions with obstacles
        for (const auto &obstacle : env) {
            std::vector<Point> obstacleCorners = getSegmentCorners(obstacle);
            
            for (const auto &segment : link_segments) {
                for (int j = 0; j < 4; ++j) {
                    Point C = obstacleCorners[j];
                    Point D = obstacleCorners[(j + 1) % 4];

                    if (doSegmentsIntersect(segment.first, segment.second, C, D)) {
                        return false;
                    }
                }
            }
        }

        return true;
    });
}

int main(int argc, char **argv) {
    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 2);

    switch (scenario) {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    setupCollisionChecker(ss, env);

    // Setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario) {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss, env);
            benchScenario2(ss, env);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    return 0;
}