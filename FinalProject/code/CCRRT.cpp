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
#include <cmath>
#include <fstream>
#include <functional>
#include <vector>


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

// CCRRT (Chance Constrained RRT) extends the standard RRT algorithm 
// to handle uncertainty in motion planning by maintaining state uncertainty 
// and checking probabilistic collision constraints

class CCRRT : public oc::RRT {
public:
    struct StateWithCovariance {
        Eigen::VectorXd mean; // Mean vector of the state
        Eigen::MatrixXd covariance; // Covariance matrix of the state
    };

    // Constructor initializes the planner with a safety probability threshold
    CCRRT(const oc::SpaceInformationPtr &si, double psafe = 0.99) 
        : oc::RRT(si), psafe_(psafe) {
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
        Eigen::MatrixXd Pw = Eigen::MatrixXd::Identity(dim, dim) * 0.01;  // Process noise covariance

        // Propagate mean state using linear dynamics
        Eigen::VectorXd nextMean = A * startVec + B * controlVec;
        for (unsigned int i = 0; i < dim; ++i)
            rstate->values[i] = nextMean(i);

        // Propagate uncertainty using the Kalman filter prediction step
        auto startIt = stateUncertainty_.find(start);
        // If no prior uncertainty, initialize with small diagonal covariance
        Eigen::MatrixXd prevCov = (startIt != stateUncertainty_.end()) ? 
            startIt->second.covariance : 
            Eigen::MatrixXd::Identity(dim, dim) * 0.01;

        // Update covariance using linear system uncertainty propagation
        Eigen::MatrixXd nextCov = A * prevCov * A.transpose() + Pw;
        stateUncertainty_[result] = {nextMean, nextCov};
    }

    virtual void getPlannerData(ob::PlannerData &data) const override {
        oc::RRT::getPlannerData(data);
    }

protected:
    double psafe_;  // Safety probability threshold
    // Map to store state uncertainty (mean and covariance) for each visited state
    std::map<const ob::State*, StateWithCovariance> stateUncertainty_;

public:
    std::map<const ob::State*, StateWithCovariance>* getStateUncertainty() {
        return &stateUncertainty_;
    }
};

std::vector<Obstacle> obstacles;

class ChanceConstraintStateValidityChecker : public ob::StateValidityChecker {
public:
    ChanceConstraintStateValidityChecker(const ob::SpaceInformationPtr &si,
                                       double psafe,
                                       std::map<const ob::State*, CCRRT::StateWithCovariance>* stateUncertainty)
        : ob::StateValidityChecker(si), psafe_(psafe), stateUncertainty_(stateUncertainty) {}

    virtual bool isValid(const ob::State *state) const override {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        
        if (!si_->satisfiesBounds(state))
            return false;

        if (stateUncertainty_->find(state) == stateUncertainty_->end()) {
            for (const auto& obs : obstacles) {
                Eigen::Vector2d stateVec(s->values[0], s->values[1]);
                double dist = (stateVec - obs.center).norm();
                if (dist <= obs.radius)
                    return false;
            }
            return true;
        }

        const auto& uncertainty = (*stateUncertainty_)[state];
        for (const auto& obs : obstacles) {
            if (!isChanceConstraintSatisfied(uncertainty, obs))
                return false;
        }
        return true;
    }
    
private:
    double psafe_;
    std::map<const ob::State*, CCRRT::StateWithCovariance>* stateUncertainty_;

    bool isChanceConstraintSatisfied(const CCRRT::StateWithCovariance& stateUnc,
                                   const Obstacle& obs) const {
        Eigen::Vector2d mean = stateUnc.mean.head<2>();
        Eigen::Matrix2d cov = stateUnc.covariance.block<2,2>(0, 0);
        Eigen::Vector2d diff = obs.center - mean;
        double dist = diff.norm();
        
        double sigma = std::sqrt((diff.transpose() * cov * diff)(0)) / dist;
        double beta = std::sqrt(2) * erfcinv(2 * (1 - psafe_));
        return dist > obs.radius + beta * sigma;
    }
};

int main(int argc, char **argv)
{
    // 1) Set up state & control spaces
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->setBounds(cbounds);

    // 2) Create SpaceInformation & CCRRT planner
    auto si      = std::make_shared<oc::SpaceInformation>(space, cspace);
    auto planner = std::make_shared<CCRRT>(si, /*psafe=*/0.99);

    // 3) Use CCRRT::propagate so we track covariance
    si->setStatePropagator(
        [planner](const ob::State *start, const oc::Control *ctrl,
                  double dt, ob::State *result) {
            planner->propagate(start, ctrl, dt, result);
        });

    // 4) Static obstacles & write obstacles.txt
    obstacles.push_back({{ 5.0,  5.0}, 1.0});
    obstacles.push_back({{-3.0,  2.0}, 1.5});
    {
        std::ofstream f("obstacles.txt");
        for (auto &obs : obstacles)
            f << obs.center[0]-obs.radius << ","
              << obs.center[1]-obs.radius << ","
              << 2*obs.radius << "," << 2*obs.radius << "\n";
    }

    // 5) Start / goal
    ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
    start[0] = -5; start[1] = -5;
    goal[0]  =  7; goal[1]  =  7;

    // 6) SimpleSetup
    oc::SimpleSetup ss(si);
    ss.setPlanner(planner);

    // 7) Collision + chance‐constraint checker
    si->setStateValidityChecker(
        std::make_shared<ChanceConstraintStateValidityChecker>(
            si, /*psafe=*/0.99, planner->getStateUncertainty()));

    // 8) Motion validator
    auto mv = std::make_shared<CCRRTMotionValidator>(si, /*psafe=*/0.99);
    mv->setObstacles(obstacles);
    si->setMotionValidator(mv);

    // 9) Tuning
    si->setStateValidityCheckingResolution(0.01);
    si->setPropagationStepSize(0.05);

    ss.setStartAndGoalStates(start, goal, /*threshold=*/0.5);

    // 10) Plan!
    ss.solve(ob::timedPlannerTerminationCondition(5.0));

    if (!ss.haveSolutionPath())
    {
        std::cout << "No solution found\n";
        return 0;
    }

    //
    // --- Write out the *smoothed* solution path
    //
    {
        std::ofstream pathFile("solution_path.txt");
        auto smooth = ss.getSolutionPath().asGeometric();
        smooth.interpolate();
        for (std::size_t i = 0; i < smooth.getStateCount(); ++i)
        {
            auto *st = smooth.getState(i)
                           ->as<ob::RealVectorStateSpace::StateType>();
            pathFile << st->values[0] << " "
                     << st->values[1] << " 0 0\n";
        }
        std::cout << "Path written to solution_path.txt\n";
    }

    //
    // --- Dump covariances from the *raw* (un‐interpolated) path
    //
    {
        std::ofstream covFile("covariances.txt");
        std::cout << "[INFO] Writing covariances...\n";
        auto raw = ss.getSolutionPath().asGeometric();
        auto smooth = ss.getSolutionPath().asGeometric(); 
        smooth.interpolate();
        std::size_t N = raw.getStateCount();
        std::cout << "[INFO] Raw path has " << N << " states.\n";

        auto *uncert = planner->getStateUncertainty();
        for (std::size_t i = 0; i < N; ++i)
        {
            const ob::State *s = raw.getState(i);
            auto it = uncert->find(s);
            if (it != uncert->end())
            {
                const auto &C = it->second.covariance;
                covFile << C(0,0) << " " << C(0,1) << " "
                        << C(1,0) << " " << C(1,1) << "\n";
            }
            else
            {
                covFile << "0 0 0 0\n";
            }
        }
        std::cout << "[INFO] Covariances written to covariances.txt\n";
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
