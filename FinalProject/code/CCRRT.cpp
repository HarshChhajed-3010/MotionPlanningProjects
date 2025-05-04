#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Exception.h>
#include <Eigen/Dense>
#include "MotionValidator.h"
#include "Obstacles.h"
#include <cmath>
#include <fstream>
#include <functional>
#include <vector>

// Define global obstacle containers
std::vector<Obstacle> staticObstacles;
std::vector<DynamicObstacle> dynamicObstacles;

// Implementation of inverse complementary error function
inline double erfcinv(double x) {
    if (x >= 2.0) return -std::numeric_limits<double>::infinity();
    if (x <= 0.0) return std::numeric_limits<double>::infinity();

    const double pp = (x < 1.0) ? x : 2.0 - x;
    const double t = std::sqrt(-2.0 * std::log(pp / 2.0));
    double p = -0.70711 * ((2.30753 + t * 0.27061) / (1.0 + t * (0.99229 + t * 0.04481)) - t);

    for (int i = 0; i < 2; i++) {
        double err = std::erfc(p) - pp;
        p += err / (1.12837916709551257 * std::exp(-p * p) - p * err);
    }
    return (x < 1.0) ? p : -p;
}

using namespace ompl::base;
using namespace ompl::control;

namespace ob = ompl::base;
namespace oc = ompl::control;

// Add this struct at the top (after includes)
struct StateKey {
    double x, y;
    bool operator<(const StateKey& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    static StateKey fromState(const ob::State* s) {
        auto* st = s->as<ob::RealVectorStateSpace::StateType>();
        return {st->values[0], st->values[1]};
    }
};

// CCRRT (Chance Constrained RRT) extends the standard RRT algorithm 
// to handle uncertainty in motion planning by maintaining state uncertainty 
// and checking probabilistic collision constraints

class CCRRT : public oc::RRT {
public:
    struct StateWithCovariance {
        Eigen::VectorXd mean; // Mean vector of the state
        Eigen::MatrixXd covariance; // Covariance matrix of the state
        double timestamp;  // Time when this state was reached
    };

    // Constructor initializes the planner with a safety probability threshold
    CCRRT(const oc::SpaceInformationPtr &si, double psafe = 0.99) 
        : oc::RRT(si), psafe_(psafe), currentTime_(0.0) {
        setName("CCRRT");
    }

    // The propagate function implements the state transition model
    // It updates both the nominal state and its associated uncertainty
    void propagate(const ob::State *start, const oc::Control *control,
                  const double duration, ob::State *result) {
        // Get the control space information
        auto csi = std::static_pointer_cast<oc::SpaceInformation>(si_);
        si_->getStateSpace()->copyState(result, start);
    
        const unsigned int dim = si_->getStateDimension();
        const unsigned int ctrl_dim = csi->getControlSpace()->getDimension();
        
        // Extract control values from RealVectorControlSpace
        const auto *rctrl = control->as<oc::RealVectorControlSpace::ControlType>();
        auto *rstate = result->as<ob::RealVectorStateSpace::StateType>();

        // Convert states and controls to Eigen vectors for easier manipulation
        Eigen::VectorXd startVec(dim);
        Eigen::VectorXd controlVec(ctrl_dim);

        for (unsigned int i = 0; i < dim; ++i)
            startVec(i) = start->as<ob::RealVectorStateSpace::StateType>()->values[i];

        for (unsigned int i = 0; i < ctrl_dim; ++i)
            controlVec(i) = rctrl->values[i];

        // Linear system matrices for uncertainty propagation
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim, dim);  // State transition matrix
        Eigen::MatrixXd B = Eigen::MatrixXd::Identity(dim, ctrl_dim);  // Control input matrix
        Eigen::MatrixXd Pw = Eigen::MatrixXd::Identity(dim, dim) * 0.01;  // Process noise covariancee (reduced)

        // Propagate mean state using linear dynamics (scaled by duration)
        Eigen::VectorXd nextMean = A * startVec + B * controlVec * duration;
        for (unsigned int i = 0; i < dim; ++i)
            rstate->values[i] = nextMean(i);

        // Propagate uncertainty using the Kalman filter prediction step
        auto startIt = stateUncertainty_.find(StateKey::fromState(start));
        // If no prior uncertainty, initialize with process noise (Pw) instead of small diagonal
        Eigen::MatrixXd prevCov = (startIt != stateUncertainty_.end()) ? 
            startIt->second.covariance : 
            Pw; // Replace 0.01*I with Pw
            
        // Get current timestamp or initialize to 0
        double currentTime = (startIt != stateUncertainty_.end()) ? 
            startIt->second.timestamp + duration : 
            currentTime_ + duration;
            
        // Update current time to max observed time
        currentTime_ = std::max(currentTime_, currentTime);

        // Propagate covariance: nextCov = A*prevCov*A^T + Pw
        Eigen::MatrixXd nextCov = A * prevCov * A.transpose() + Pw;
        stateUncertainty_[StateKey::fromState(result)] = {nextMean, nextCov, currentTime};

        // Add additional uncertainty when near dynamic obstacles
        for (const auto& dynObs : dynamicObstacles) {
            Eigen::Vector2d obsPos = dynObs.getPositionAtTime(currentTime);
            Eigen::Vector2d statePos(nextMean(0), nextMean(1));
            double dist = (obsPos - statePos).norm();
    
            if (dist < 5.0) { // If within 5 units of a dynamic obstacle
                double scaling = (5.0 - dist)/5.0; // Scale from 0 to 1
                Eigen::Matrix2d obsUncertainty = Eigen::Matrix2d::Identity() * 
                                                dynObs.getPositionUncertainty() * 
                                                scaling;
                nextCov.block<2,2>(0,0) += obsUncertainty;
            }
        }
    }

    virtual void getPlannerData(ob::PlannerData &data) const override {
        oc::RRT::getPlannerData(data);
    }

protected:
    double psafe_;  // Safety probability threshold
    double currentTime_;  // Current simulation time
    // Map to store state uncertainty (mean, covariance, timestamp) for each visited state
    std::map<StateKey, StateWithCovariance> stateUncertainty_;

public:
    std::map<StateKey, StateWithCovariance>* getStateUncertainty() {
        return &stateUncertainty_;
    }
    
    double getCurrentTime() const {
        return currentTime_;
    }
};

class ChanceConstraintStateValidityChecker : public ob::StateValidityChecker {
public:
    ChanceConstraintStateValidityChecker(const ob::SpaceInformationPtr &si,
                                       double psafe,
                                       std::map<StateKey, CCRRT::StateWithCovariance>* stateUncertainty)
        : ob::StateValidityChecker(si), psafe_(psafe), stateUncertainty_(stateUncertainty) {}

    virtual bool isValid(const ob::State *state) const override {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        
        if (!si_->satisfiesBounds(state))
            return false;

        // Get time for dynamic obstacle position
        double time = 0.0;
        auto key = StateKey::fromState(state);
        if (stateUncertainty_->find(key) != stateUncertainty_->end()) {
            time = (*stateUncertainty_)[key].timestamp;
        }

        // Check static obstacles (treat rectangles as circles for collision)
        for (const auto& obs : staticObstacles) {
            Eigen::Vector2d stateVec(s->values[0], s->values[1]);
            if (obs.isCircular()) {
                double dist = (stateVec - obs.getCenter()).norm();
                if (dist <= obs.getRadius())
                    return false;
            } else {
                // Rectangle: use circumscribed circle
                Eigen::Vector2d obsCenter = obs.getCenter();
                double obsRadius = std::hypot(obs.getWidth()/2, obs.getHeight()/2);
                double dist = (stateVec - obsCenter).norm();
                if (dist <= obsRadius)
                    return false;
            }
        }
        
        // Check dynamic obstacles at their position at time t
        for (const auto& dynObs : dynamicObstacles) {
            Eigen::Vector2d obsPosition = dynObs.getPositionAtTime(time);
            Eigen::Vector2d stateVec(s->values[0], s->values[1]);
            double dist = (stateVec - obsPosition).norm();
            if (dist <= dynObs.getRadius())
                return false;
        }

        // If we have uncertainty information, check probabilistic constraints
        if (stateUncertainty_->find(key) != stateUncertainty_->end()) {
            const auto& uncertainty = (*stateUncertainty_)[key];
            
            // Check against static obstacles (treat rectangles as circles)
            for (const auto& obs : staticObstacles) {
                if (obs.isCircular()) {
                    if (!isChanceConstraintSatisfied(uncertainty, obs))
                        return false;
                } else {
                    // Rectangle: use circumscribed circle
                    Eigen::Vector2d obsCenter = obs.getCenter();
                    double obsRadius = std::hypot(obs.getWidth()/2, obs.getHeight()/2);
                    Obstacle circObs(obsCenter[0], obsCenter[1], obsRadius);
                    if (!isChanceConstraintSatisfied(uncertainty, circObs))
                        return false;
                }
            }
            
            // Check against dynamic obstacles at their position at time t
            for (const auto& dynObs : dynamicObstacles) {
                Eigen::Vector2d pos = dynObs.getPositionAtTime(time);
                Obstacle tempObs(pos[0], pos[1], dynObs.getRadius());
                if (!isChanceConstraintSatisfied(uncertainty, tempObs))
                    return false;
            }
        }
        
        return true;
    }
    
private:
    double psafe_;
    std::map<StateKey, CCRRT::StateWithCovariance>* stateUncertainty_;

    bool isChanceConstraintSatisfied(const CCRRT::StateWithCovariance& stateUnc,
                                   const Obstacle& obs) const {
        Eigen::Vector2d mean = stateUnc.mean.head<2>();
        Eigen::Matrix2d cov = stateUnc.covariance.block<2,2>(0, 0);
        Eigen::Vector2d diff = obs.getCenter() - mean;
        double dist = diff.norm();
        double directionalVar = (dist > 1e-6) ? 
            (diff.normalized().transpose() * cov * diff.normalized()) :
            cov.eigenvalues().real().maxCoeff();
        double sigma = std::sqrt(directionalVar);
        double beta = std::sqrt(2) * erfcinv(2 * (1 - psafe_));
        return dist > obs.getRadius() + beta * sigma;
    }
};

// Helper function to write obstacle states at different time points
void writeObstacleTrajectory(const std::vector<DynamicObstacle>& dynObstacles, 
                           double maxTime, double timeStep) {
    std::ofstream trajFile("obstacle_trajectory.txt");
    
    for (double t = 0; t <= maxTime; t += timeStep) {
        for (size_t i = 0; i < dynObstacles.size(); ++i) {
            Eigen::Vector2d pos = dynObstacles[i].getPositionAtTime(t);
            double radius = dynObstacles[i].getRadius();
            
            trajFile << t << " " << i << " " 
                    << pos[0] << " " << pos[1] << " " 
                    << radius << "\n";
        }
    }
}

// Helper: Find nearest state in uncertainty map by Euclidean distance
const CCRRT::StateWithCovariance* findNearestUncertainty(
    const std::map<StateKey, CCRRT::StateWithCovariance>& uncert,
    const ob::State* query)
{
    if (uncert.empty() || query == nullptr) {
        std::cerr << "[DEBUG] Uncertainty map empty or query is null\n";
        return nullptr;
    }
    StateKey qk = StateKey::fromState(query);
    double minDist = std::numeric_limits<double>::max();
    const CCRRT::StateWithCovariance* nearest = nullptr;
    for (const auto& pair : uncert) {
        double dx = qk.x - pair.first.x;
        double dy = qk.y - pair.first.y;
        double dist = dx*dx + dy*dy;
        if (dist < minDist) {
            minDist = dist;
            nearest = &pair.second;
        }
    }
    if (!nearest) {
        std::cerr << "[DEBUG] No nearest state found in uncertainty map\n";
    }
    return nearest;
}

// ...existing code...

int main(int argc, char **argv)
{
    // 1) Set up state & control spaces
    // State space: R2 (2D Euclidean space for (x, y) position)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    // Control space: R2 (2D Euclidean space for (vx, vy) control inputs)
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-2);   // Increased from -1 to -2
    cbounds.setHigh(2);   // Increased from 1 to 2
    cspace->setBounds(cbounds);

    // 2) Create SpaceInformation & CCRRT planner
    auto si      = std::make_shared<oc::SpaceInformation>(space, cspace);
    auto planner = std::make_shared<CCRRT>(si, /*psafe=*/0.98);

    // Set min/max control duration to avoid OMPL warning
    si->setMinMaxControlDuration(1, 50);

    // Set the goal bias - add this line
    planner->setGoalBias(0.1); // 5% chance to sample the goal

    // 3) Use CCRRT::propagate so we track covariance
    si->setStatePropagator(
        [planner](const ob::State *start, const oc::Control *ctrl,
                  double dt, ob::State *result) {
            planner->propagate(start, ctrl, dt, result);
        });

    // 4) Set up obstacles
    // Static obstacle (the square at (4,4))
    staticObstacles.push_back(Obstacle(4.0, 4.0, 1.0));
    
    // Convert the rectangular obstacle to dynamic
    // Initial position: (-4.0, 1.0)
    // Final position: (4.0, 1.0) - moves to the right
    // Velocity: Moving right at 0.5 units per secondd
    dynamicObstacles.push_back(DynamicObstacle(
        Eigen::Vector2d(-4.0, 1.0),  // Initial center
        1.5,                         // Radius
        Eigen::Vector2d(0.15, 0.0),  // Velocity
        Eigen::Vector2d(4.0, 1.0)    // Final position
    ));

    // Convert the rectangular obstacle to dynamic
    // Initial position: (-4.0, 1.0)
    // Final position: (-4.0, 7.0) - moves upward
    // Velocity: Moving up at 0.15 units per second
    dynamicObstacles.push_back(DynamicObstacle(
        Eigen::Vector2d(-4.0, 1.0),  // Initial center
        1.5,                         // Radius
        Eigen::Vector2d(0.0, 0.15),  // Velocity (upward)
        Eigen::Vector2d(-4.0, 7.0)   // Final position (upward)
    ));

    // Add a dynamic obstacle moving downward
    // Initial position: (4.0, 7.0)
    // Final position: (4.0, 1.0) - moves downward
    // Velocity: Moving down at 0.15 units per second
    dynamicObstacles.push_back(DynamicObstacle(
        Eigen::Vector2d(7.0, 7.0),   // Initial center
        1.5,                         // Radius
        Eigen::Vector2d(0.0, -0.15), // Velocity (downward)
        Eigen::Vector2d(7.0, -5.0)    // Final position (downward)
    ));

    // Write obstacle definitions to file for visualization
    {
        std::ofstream f("obstacles.txt");
        // Static obstacles
        for (auto &obs : staticObstacles)
            f << obs.getX()-obs.getRadius() << ","
              << obs.getY()-obs.getRadius() << ","
              << 2*obs.getRadius() << "," << 2*obs.getRadius() << "\n";
              
        // Initial position of dynamic obstacles
        for (auto &obs : dynamicObstacles)
            f << obs.getX()-obs.getRadius() << ","
              << obs.getY()-obs.getRadius() << ","
              << 2*obs.getRadius() << "," << 2*obs.getRadius() << " dynamic\n";
    }

    // 5) Start / goal
    // Start and goal states are in R2
    ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
    start[0] = -5; start[1] = -5;
    goal[0]  =  7; goal[1]  =  7;

    // 6) SimpleSetup
    oc::SimpleSetup ss(si);
    ss.setPlanner(planner);

    // 7) Collision + chance‐constraint checker
    si->setStateValidityChecker(
        std::make_shared<ChanceConstraintStateValidityChecker>(
            si, /*psafe=*/0.98, planner->getStateUncertainty()));

    // 8) Motion validator
    auto mv = std::make_shared<CCRRTMotionValidator>(si, /*psafe=*/0.98);
    // Convert dynamic obstacles to static for motion validator
    // This is ok since motion validator works with small segments
    std::vector<Obstacle> allObsForValidator = staticObstacles;
    for (const auto& dynObs : dynamicObstacles) {
        Eigen::Vector2d pos = dynObs.getPositionAtTime(0);
        allObsForValidator.push_back(Obstacle(pos[0], pos[1], dynObs.getRadius()));
    }
    mv->setObstacles(allObsForValidator);
    si->setMotionValidator(mv);

    // 9) Tuning
    si->setStateValidityCheckingResolution(0.02);
    si->setPropagationStepSize(0.1);

    ss.setStartAndGoalStates(start, goal, /*threshold=*/1.0);

    // 10) Plan!
    ss.solve(ob::timedPlannerTerminationCondition(10.0));

    if (!ss.haveSolutionPath())
    {
        std::cout << "No solution found\n";
        return 0;
    }

    // Write the obstacle trajectory for visualization
    writeObstacleTrajectory(dynamicObstacles, planner->getCurrentTime(), 0.1);

    //
    // --- Write out the *smoothed* solution path
    //
    {
        std::ofstream pathFile("solution_path.txt");
        auto smooth = ss.getSolutionPath().asGeometric();
        smooth.interpolate(50); // Interpolate to 50 states
        for (std::size_t i = 0; i < smooth.getStateCount(); ++i)
        {
            auto *st = smooth.getState(i)
                           ->as<ob::RealVectorStateSpace::StateType>();
            pathFile << st->values[0] << " "
                     << st->values[1] << " 0 0\n";
        }
        std::cout << "Path written to solution_path.txt\n";
    }

    
    {
        std::ofstream covFile("covariances.txt");
        std::cout << "[INFO] Writing covariances...\n";
        auto raw = ss.getSolutionPath().asGeometric();
        std::size_t N = raw.getStateCount();
        std::cout << "[INFO] Raw path has " << N << " states.\n";
    
        auto *uncert = planner->getStateUncertainty();
        try {
            for (std::size_t i = 0; i < N; ++i) {
                const ob::State *s = raw.getState(i);
                const CCRRT::StateWithCovariance* nearest = findNearestUncertainty(*uncert, s);
                if (nearest) {
                    const auto &C = nearest->covariance;
                    covFile << C(0,0) << " " << C(0,1) << " "
                            << C(1,0) << " " << C(1,1) << "\n";
                } else {
                    covFile << "0 0 0 0\n";
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception while writing covariances: " << e.what() << std::endl;
        }
        std::cout << "[INFO] Covariances written to covariances.txt\n";
    }
    // Write time information for each state in the solution path
    {
        std::ofstream timeFile("state_times.txt");
        auto raw = ss.getSolutionPath().asGeometric();
        std::size_t N = raw.getStateCount();
        
        auto *uncert = planner->getStateUncertainty();
        for (std::size_t i = 0; i < N; ++i) {
            const ob::State *s = raw.getState(i);
            auto it = uncert->find(StateKey::fromState(s));
            if (it != uncert->end()) {
                auto *st = s->as<ob::RealVectorStateSpace::StateType>();
                timeFile << st->values[0] << " " << st->values[1] << " " 
                         << it->second.timestamp << "\n";
            }
        }
        std::cout << "[INFO] State times written to state_times.txt\n";
    }
    //
    // --- Dump the entire RRT search tree (all tried edges)
    //
    {
        ompl::base::PlannerData pdata(si);
        ss.getPlannerData(pdata);

        std::ofstream treeFile("rrt_tree.txt");
        for (unsigned int vid = 0; vid < pdata.numVertices(); ++vid)
        {
            std::vector<unsigned int> edges;
            pdata.getEdges(vid, edges);
            for (auto to : edges)
            {
                const auto *su = pdata.getVertex(vid).getState()
                                   ->as<ob::RealVectorStateSpace::StateType>();
                const auto *sv = pdata.getVertex(to).getState()
                                   ->as<ob::RealVectorStateSpace::StateType>();
                treeFile
                  << su->values[0] << " " << su->values[1]
                  << "   "
                  << sv->values[0] << " " << sv->values[1]
                  << "\n";
            }
        }
        treeFile.close();
        std::cout << "RRT tree written to rrt_tree.txt\n";
    }
    return 0;
}