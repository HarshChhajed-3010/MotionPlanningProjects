/*********************************************************************
 * PlanningRTP.cpp
 *
 * This file sets up planning problems for a point robot and a rigid
 * square robot in 2D. Two environments are provided by filling a vector
 * of axis-aligned rectangular obstacles. The planning is performed using
 * the RTP planner (RTP.h/RTP.cpp). The resulting obstacles and solution
 * path are written to files ("obstacles.txt" and "path.txt") for visualization.
 *
 * Author: Pranay Katyal, Harsh Chhajed
 *********************************************************************/

 #include <iostream>
 #include <vector>
 #include <fstream>
 #include <cmath>
 #include <limits>
 #include <algorithm>
 
 // OMPL includes
 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/ProblemDefinition.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/geometric/PathGeometric.h>
 #include <ompl/util/Console.h>
 #include <ompl/base/ScopedState.h>
 #include <ompl/base/PlannerStatus.h>
 #include <ompl/base/PlannerTerminationCondition.h>
 
 // Our planner and collision checking
 #include "RTP.h"
 #include "CollisionChecking.h"  // Defines: struct Rectangle { double x, y, width, height; }
 
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
 
 // -------------------------------------------------------------------
 // Helper: Polygon intersection using the Separating Axis Theorem (SAT)
 // -------------------------------------------------------------------
 namespace {
 bool polygonsIntersect(const std::vector<std::pair<double, double>> &poly1,
                        const std::vector<std::pair<double, double>> &poly2)
 {
     auto getAxes = [](const std::vector<std::pair<double, double>> &poly) -> std::vector<std::pair<double, double>> {
         std::vector<std::pair<double, double>> axes;
         size_t n = poly.size();
         for (size_t i = 0; i < n; ++i)
         {
             double dx = poly[(i + 1) % n].first - poly[i].first;
             double dy = poly[(i + 1) % n].second - poly[i].second;
             double len = std::sqrt(dx * dx + dy * dy);
             if (len > 1e-6)
                 axes.push_back({-dy / len, dx / len});
         }
         return axes;
     };
 
     std::vector<std::pair<double, double>> axes1 = getAxes(poly1);
     std::vector<std::pair<double, double>> axes2 = getAxes(poly2);
     axes1.insert(axes1.end(), axes2.begin(), axes2.end());
 
     for (const auto &axis : axes1)
     {
         double min1 = std::numeric_limits<double>::infinity();
         double max1 = -std::numeric_limits<double>::infinity();
         for (const auto &p : poly1)
         {
             double proj = p.first * axis.first + p.second * axis.second;
             min1 = std::min(min1, proj);
             max1 = std::max(max1, proj);
         }
         double min2 = std::numeric_limits<double>::infinity();
         double max2 = -std::numeric_limits<double>::infinity();
         for (const auto &p : poly2)
         {
             double proj = p.first * axis.first + p.second * axis.second;
             min2 = std::min(min2, proj);
             max2 = std::max(max2, proj);
         }
         if (max1 < min2 || max2 < min1)
             return false;
     }
     return true;
 }
 }  // anonymous namespace
 
 // -------------------------------------------------------------------
 // Function to plan for a point robot in 2D
 // -------------------------------------------------------------------
 void planPoint(const std::vector<Rectangle> &obstacles, int envID) {
    std::string baseFilename = "Point_Robot_Environment_" + std::to_string(envID);

    OMPL_INFORM("Planning for point robot in 2D...");

    // Setup state space: R^2
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Create space information.
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([&obstacles](const ob::State *state) -> bool {
        const auto *rvState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = rvState->values[0];
        double y = rvState->values[1];
        for (const auto &rect : obstacles) {
            if (x >= rect.x && x <= rect.x + rect.width &&
                y >= rect.y && y <= rect.y + rect.height)
                return false;
        }
        return true;
    });
    si->setup();

    // Define start and goal states.
    ob::ScopedState<> start(space);
    start[0] = 1.0;
    start[1] = 1.0;
    ob::ScopedState<> goal(space);
    goal[0] = 9.0;
    goal[1] = 9.0;

    // Create problem definition.
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal, 0.001);

    // Create and configure RTP planner.
    ompl::geometric::RTP planner(si);
    planner.setProblemDefinition(pdef);

    planner.setup();

    // Solve with a 30-second timeout.
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(30.0);
    ob::PlannerStatus solved = planner.solve(ptc);

    if (solved) {
        OMPL_INFORM("Point robot: solution found.");
        std::ofstream obsFile(baseFilename + "_obstacles.txt");
        for (const auto &rect : obstacles)
            obsFile << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << std::endl;
        obsFile.close();

        std::ofstream pathFile(baseFilename + "_path.txt");
        pathFile << "R2" << std::endl;
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            const auto *state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            pathFile << state->values[0] << " " << state->values[1] << std::endl;
        }
        pathFile.close();
    } else {
        OMPL_INFORM("Point robot: no solution found.");
    }
}
 // -------------------------------------------------------------------
 // Function to plan for a rigid box robot in SE2
 // -------------------------------------------------------------------
 void planBox(const std::vector<Rectangle> &obstacles, int envID) {
    std::string baseFilename = "Box_Robot_Environment_" + std::to_string(envID);

    OMPL_INFORM("Planning for rigid box robot in SE2...");

    // Setup state space: SE(2)
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    ob::RealVectorBounds rbounds(2);
    rbounds.setLow(0);
    rbounds.setHigh(10);
    space->as<ob::SE2StateSpace>()->setBounds(rbounds);

    // Create space information.
    auto si = std::make_shared<ob::SpaceInformation>(space);
    const double side = 1.0;  // side length of the box

    si->setStateValidityChecker([obstacles, side, rbounds](const ob::State *state) -> bool {
        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
        double x = se2state->getX();
        double y = se2state->getY();
        double theta = se2state->getYaw();

        double half = side / 2.0;
        std::vector<std::pair<double, double>> boxCorners;
        std::vector<std::pair<double, double>> localCorners = {
            {-half, -half},
            { half, -half},
            { half,  half},
            {-half,  half}
        };
        for (const auto &lc : localCorners) {
            double rx = std::cos(theta) * lc.first - std::sin(theta) * lc.second;
            double ry = std::sin(theta) * lc.first + std::cos(theta) * lc.second;
            boxCorners.push_back({x + rx, y + ry});
        }
        for (const auto &corner : boxCorners) {
            if (corner.first < rbounds.low[0] || corner.first > rbounds.high[0] ||
                corner.second < rbounds.low[1] || corner.second > rbounds.high[1])
                return false;
        }
        for (const auto &rect : obstacles) {
            std::vector<std::pair<double, double>> rectPoly = {
                {rect.x, rect.y},
                {rect.x + rect.width, rect.y},
                {rect.x + rect.width, rect.y + rect.height},
                {rect.x, rect.y + rect.height}
            };
            if (polygonsIntersect(boxCorners, rectPoly))
                return false;
        }
        return true;
    });
    si->setup();

    // Define start and goal states.
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(1.0);
    start->setY(1.0);
    start->setYaw(0.0);
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(9.0);
    goal->setY(9.0);
    goal->setYaw(0.0);

    // Create problem definition.
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal, 0.001);

    // Create and configure RTP planner.
    ompl::geometric::RTP planner(si);
    planner.setProblemDefinition(pdef);
    planner.setup();

    // Solve with a 30-second timeout.
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(30.0);
    ob::PlannerStatus solved = planner.solve(ptc);

    if (solved) {
        OMPL_INFORM("Box robot: solution found.");
        std::ofstream obsFile(baseFilename + "_obstacles.txt");
        for (const auto &rect : obstacles)
            obsFile << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << std::endl;
        obsFile.close();

        std::ofstream pathFile(baseFilename + "_path.txt");
        pathFile << "SE2 " << side << std::endl;  // Include configuration space and robot size
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            const auto *state = path->getState(i)->as<ob::SE2StateSpace::StateType>();
            pathFile << state->getX() << " " << state->getY() << " " << state->getYaw() << std::endl;
        }
        pathFile.close();
    } else {
        OMPL_INFORM("Box robot: no solution found.");
    }
}
 // -------------------------------------------------------------------
 // Environment definitions
 // -------------------------------------------------------------------
 void makeEnvironment1(std::vector<Rectangle> &obstacles)
 {
     // Example: Two obstacles.
     obstacles.push_back({3.0, 3.0, 1.0, 4.0});
     obstacles.push_back({6.0, 5.0, 3.0, 2.0});
 }
 
 void makeEnvironment2(std::vector<Rectangle> &obstacles)
 {
     // Example: Three obstacles.
     obstacles.push_back({2.0, 2.0, 1.0, 3.0});
     obstacles.push_back({5.0, 5.0, 3.0, 3.0});
     obstacles.push_back({8.0, 0.0, 1.0, 4.0});
 }
 
 // -------------------------------------------------------------------
 // Main function
 // -------------------------------------------------------------------

 int main(int /*argc*/, char ** /*argv*/)
 {
     for (int envID = 1; envID <= 2; ++envID)
     {
         std::vector<Rectangle> obstacles;
         if (envID == 1)
             makeEnvironment1(obstacles);
         else
             makeEnvironment2(obstacles);
 
         planPoint(obstacles, envID);
         planBox(obstacles, envID);
     }
 
     return 0;
 }