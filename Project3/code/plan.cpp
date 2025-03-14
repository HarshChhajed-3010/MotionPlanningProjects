#include "KinematicChain.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

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

// This code for Scenario2

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


void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -0.5*M_PI;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);

}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
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

    //Obstacle 1
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
        std::ofstream pathFile("narrow_path.txt");
        ss.getSolutionPath().printAsMatrix(pathFile);
        pathFile.close();
        std::cout << "Solution saved to narrow.txt\n";
    } else {
        std::cout << "No solution found.\n";
    }
}


void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");

    // RRT Connect
    auto rrt_connect = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
    rrt_connect->setProblemDefinition(ss.getProblemDefinition());
    rrt_connect->setName("RRT_connect");
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

    ompl::tools::Benchmark::Request request(20.0, 1024.0, 10, 0.5);
    b.benchmark(request);
    b.saveResultsToFile("benchmarkScenario1.log");
}



void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    

    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    planner->setRange(0.2);  // Limits max step size
    

    planner->setGoalBias(0.1); // Default is usually 0.05
    
    ss.setPlanner(planner);

    if (ss.solve(100.0)) {
        std::ofstream pathFile("clear_path.txt");
        ss.getSolutionPath().printAsMatrix(pathFile);
        pathFile.close();
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "No solution found.\n";
    }
}

void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    auto rrt_star = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    rrt_star->setProblemDefinition(ss.getProblemDefinition());
    rrt_star->setName("RRT*");
    b.addPlanner(rrt_star);

    auto prm_star = std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation());
    prm_star->setProblemDefinition(ss.getProblemDefinition());
    prm_star->setName("PRM*");
    b.addPlanner(prm_star);

    auto rrt_sharp = std::make_shared<ompl::geometric::RRTsharp>(ss.getSpaceInformation());
    rrt_sharp->setProblemDefinition(ss.getProblemDefinition());
    rrt_sharp->setName("RRT#");
    b.addPlanner(rrt_sharp);

    ompl::tools::Benchmark::Request request(60.0, 1024.0, 10, 0.5);
    b.benchmark(request);
    b.saveResultsToFile("benchmarkScenario2.log");
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



std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{
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

// // Struct and class definitions (for those that don't already exist)
// struct Point {
//     double x, y;
//     Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
// };

// struct Line {
//     double a, b, c;
//     Line(const Point& p1, const Point& p2) {
//         a = p2.y - p1.y;
//         b = p1.x - p2.x;
//         c = a * p1.x + b * p1.y;
//     }
// };

// struct Rectangle {
//     double x, y, width, height;
// };

// Note: Not redefining Segment as it's already defined in KinematicChain.h

// Helper functions
Point rotatePoint(const Point& p, double angle) {
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);
    return Point(
        p.x * cosAngle - p.y * sinAngle,
        p.x * sinAngle + p.y * cosAngle
    );
}

std::vector<Point> getSquareCorners(const Point& center, double sideLength, double angle) {
    double halfSide = sideLength / 2.0;

    std::vector<Point> corners = {
        Point(-halfSide, -halfSide),
        Point(halfSide, -halfSide),
        Point(halfSide, halfSide),
        Point(-halfSide, halfSide)
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

// Assume Segment is already defined with x0, y0, x1, y1 fields
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

bool doSegmentsIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
    auto orientation = [](const Point& a, const Point& b, const Point& c) {
        double val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
        if (val > 0) return 1;  // Clockwise
        if (val < 0) return -1; // Counterclockwise
        return 0;               // Collinear
    };

    auto onSegment = [](const Point& p, const Point& q, const Point& r) {
        return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
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

// Modified to accept Environment (vector<Segment>) 
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

// Functions to convert between std::pair<double, double> and Point
Point pairToPoint(const std::pair<double, double>& p) {
    return Point(p.first, p.second);
}

std::pair<double, double> pointToPair(const Point& p) {
    return {p.x, p.y};
}

//START OF CUSTOM FUNCTION

double dot(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y;
}

// Function to compute the cross product of two vectors
double cross(const Point& a, const Point& b) {
    return a.x * b.y - a.y * b.x;
}

// Function to compute the distance between two points
double distance(const Point& a, const Point& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Function to calculate the distance from point p to segment ab
double pointToSegmentDistance(const Point& p, const Point& a, const Point& b) {
    // Vector AB and AP
    Point ab = {b.x - a.x, b.y - a.y};
    Point ap = {p.x - a.x, p.y - a.y};

    // Project point p onto the line through a and b
    double ab2 = dot(ab, ab);  // length squared of AB
    if (ab2 == 0) return distance(p, a);  // a and b are the same point
    double t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));

    // Closest point on the segment
    Point projection = {a.x + t * ab.x, a.y + t * ab.y};
    return distance(p, projection);
}

double minDistanceBetweenPolygons(const std::vector<Point>& poly1, const std::vector<Point>& poly2) {
    double minDist = std::numeric_limits<double>::infinity();

    // Check every edge in poly1 against every edge in poly2
    for (size_t i = 0; i < poly1.size(); i++) {
        Point p1 = poly1[i];
        Point p2 = poly1[(i + 1) % poly1.size()];  // Next vertex (wrap around)

        for (size_t j = 0; j < poly2.size(); j++) {
            Point q1 = poly2[j];
            Point q2 = poly2[(j + 1) % poly2.size()];  // Next vertex (wrap around)

            // Calculate the distance between the segment (p1, p2) and (q1, q2)
            double dist = std::min(pointToSegmentDistance(p1, q1, q2), pointToSegmentDistance(p2, q1, q2));
            dist = std::min(dist, pointToSegmentDistance(q1, p1, p2));
            dist = std::min(dist, pointToSegmentDistance(q2, p1, p2));

            minDist = std::min(minDist, dist);
        }
    }

    return minDist;
}

// Function to check if a link segment intersects with the base
bool isLinkIntersectingBase(const Point& linkStart, const Point& linkEnd, 
                           const std::vector<Point>& baseCorners) {
    // Check intersection with all sides of the base
    for (size_t i = 0; i < baseCorners.size(); i++) {
        Point basePoint1 = baseCorners[i];
        Point basePoint2 = baseCorners[(i + 1) % baseCorners.size()];
        
        if (doSegmentsIntersect(linkStart, linkEnd, basePoint1, basePoint2)) {
            return true;
        }
    }
    return false;
}

// Function to check if kinematic chain has self-intersections
bool hasChainSelfIntersection(const std::vector<std::pair<Point, Point>>& linkSegments) {
    // Check all pairs of non-adjacent links for intersection
    for (size_t i = 0; i < linkSegments.size(); i++) {
        // Start checking from i+2 to skip adjacent links (which always "intersect" at joints)
        for (size_t j = i + 2; j < linkSegments.size(); j++) {
            // Skip checking adjacent segments
            if (j == i + 1) continue;
            
            // For the first link and last link, only check if they're not connected
            if (i == 0 && j == linkSegments.size() - 1 && 
                (distance(linkSegments[i].first, linkSegments[j].second) > 1e-9)) {
                
                if (doSegmentsIntersect(
                    linkSegments[i].first, linkSegments[i].second,
                    linkSegments[j].first, linkSegments[j].second)) {
                    return true;
                }
            }
            // For all other non-adjacent links
            else if (j >= i + 2) {
                if (doSegmentsIntersect(
                    linkSegments[i].first, linkSegments[i].second,
                    linkSegments[j].first, linkSegments[j].second)) {
                    return true;
                }
            }
        }
    }
    return false;
}

//end

void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env, double baseSize = 1.0, bool useMinDistanceCheck = false) {
    ss.setStateValidityChecker([&env, baseSize, useMinDistanceCheck](const ompl::base::State *state) {
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

        // Get base corners for collision checking
        std::vector<Point> baseCorners = getSquareCorners(Point(x, y), baseSize, theta);

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

        // Calculate the end positions of each link in the chain
        for (double joint : jointAngles) {
            angle += joint;  // Update angle
            double next_x = link_x + link_length * cos(angle);
            double next_y = link_y + link_length * sin(angle);

            // Create link segment (start and end points)
            Point linkStart(link_x, link_y);
            Point linkEnd(next_x, next_y);
            link_segments.push_back({linkStart, linkEnd});
            link_positions.emplace_back(next_x, next_y);

            // Check if this link intersects with the base (except for the first link which is connected to the base)
            if (link_segments.size() > 1 && isLinkIntersectingBase(linkStart, linkEnd, baseCorners)) {
                return false;
            }

            // Move to the next link's starting position
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


        // Check for self-intersection in the kinematic chain
        if (hasChainSelfIntersection(link_segments)) {
            return false;
        }

        // CHECK FOR MIN DISTANCE
        const double minClearanceThreshold = 2.0; // Minimum clearance distance
        std::vector<Point> robotBaseCorners = getSquareCorners(Point(x, y), baseSize, theta);

        for (const auto& obstacle : env) {
            // Get obstacle corners using getSegmentCorners()
            std::vector<Point> obstacleCorners = getSegmentCorners(obstacle);
            
            // Check base clearance if useMinDistanceCheck is true (scenario 2)
            if (useMinDistanceCheck) {
                // Calculate minimum distance between robot base and obstacle
                double baseMinDist = minDistanceBetweenPolygons(robotBaseCorners, obstacleCorners);
                if (baseMinDist < minClearanceThreshold) {
                    return false; // If too close to an obstacle, invalid state
                }
            }

            // Check link clearances
            for (const auto &segment : link_segments) {
                // Convert link segment to polygon
                std::vector<Point> linkPolygon = {
                    segment.first, 
                    Point(segment.first.x + 0.1, segment.first.y),
                    segment.second,
                    Point(segment.second.x + 0.1, segment.second.y)
                };

                // Calculate minimum distance between link and obstacle
                double linkMinDist = minDistanceBetweenPolygons(linkPolygon, obstacleCorners);
                if (linkMinDist < minClearanceThreshold) {
                    return false; // If link is too close to an obstacle
                }
            }
        }

        // Check for direct collisions between links and obstacles
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

int main(int argc, char **argv)
{
    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
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

    // Pass true for minimum distance checking only for scenario 2
    setupCollisionChecker(ss, env, 1.0, scenario == 2);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    return 0;
}