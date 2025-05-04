// MotionValidator.cpp
#include "MotionValidator.h"
#include <ompl/control/SpaceInformation.h>
#include <cmath>

// Add these declarations
extern std::vector<Obstacle> staticObstacles;
extern std::vector<DynamicObstacle> dynamicObstacles;

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

void UncertaintyManager::storeUncertainty(const ob::State* state, 
    const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov, double timestamp) 
{
    stateUncertainty_[state] = StateUncertainty{mean, cov, timestamp};
}

void UncertaintyManager::propagateUncertainty(const ob::State* from, const ob::State* to,
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::VectorXd& control, const Eigen::MatrixXd& Pw,
    double deltaTime)
{
    auto it = stateUncertainty_.find(from);
    double currentTime = (it != stateUncertainty_.end()) ? 
        it->second.timestamp : 0.0;
    
    if (it == stateUncertainty_.end()) {
        int dim = A.rows();
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(dim);
        const auto* rs = from->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < dim; ++i)
            mean(i) = rs->values[i];
        stateUncertainty_[from] = StateUncertainty{mean, Pw, currentTime};
        it = stateUncertainty_.find(from);
    }

    // Account for relative motion in uncertainty propagation
    Eigen::VectorXd nextMean = A * it->second.mean + B * control;
    Eigen::MatrixXd nextCov = A * it->second.covariance * A.transpose() + Pw;
    
    // Add additional uncertainty due to dynamic obstacle motion
    // This increases uncertainty in the direction of obstacle motion
    Eigen::Matrix2d dynamicUncertainty = Eigen::Matrix2d::Zero();
    dynamicUncertainty(0,0) = 0.1; // Additional uncertainty in x direction due to moving obstacle
    dynamicUncertainty(1,1) = 0.1; // Add some in y as well if obstacle can move in y
    nextCov.block<2,2>(0,0) += dynamicUncertainty * deltaTime;
    stateUncertainty_[to] = StateUncertainty{nextMean, nextCov, currentTime + deltaTime};
}

bool UncertaintyManager::satisfiesChanceConstraints(const ob::State* state, 
    const std::vector<Obstacle>& obstacles) const
{
    auto it = stateUncertainty_.find(state);
    if (it == stateUncertainty_.end()) return true;
    const double currentTime = it->second.timestamp;
    Eigen::Vector2d mean = it->second.mean.head<2>();
    Eigen::Matrix2d cov = it->second.covariance.block<2,2>(0, 0);
    for (const auto& obs : obstacles) {
        if (obs.isCircular()) {
            if (!isCircleConstraintSatisfied(mean, cov, obs)) 
                return false;
        } else {
            // Rectangle: use circumscribed circle
            Eigen::Vector2d obsCenter = obs.getCenter();
            double obsRadius = std::hypot(obs.getWidth()/2, obs.getHeight()/2);
            Obstacle circObs(obsCenter[0], obsCenter[1], obsRadius);
            if (!isCircleConstraintSatisfied(mean, cov, circObs))
                return false;
        }
    }
    return true;
}

Eigen::Vector2d UncertaintyManager::getDynamicObstaclePosition(
    const Obstacle& obs, double time) const
{
    // For dynamic obstacles, compute position based on time
    // This should match the motion model in DynamicObstacle class
    if (!obs.isCircular()) {
        const double velocity = 1.0; // Matches velocity from main
        Eigen::Vector2d initialPos(obs.getX() + obs.getWidth()/2, obs.getY() + obs.getHeight()/2);
        Eigen::Vector2d displacement(velocity * time, 0.0);
        return initialPos + displacement;
    }
    return obs.getCenter();
}

bool UncertaintyManager::isCircleConstraintSatisfied(const Eigen::Vector2d& mean, 
    const Eigen::Matrix2d& cov, const Obstacle& obs) const
{
    Eigen::Vector2d diff = obs.getCenter() - mean;
    double dist = diff.norm();
    double directionalVar = (dist > 1e-6) ? 
        (diff.normalized().transpose() * cov * diff.normalized()) :
        cov.eigenvalues().real().maxCoeff();
    double sigma = std::sqrt(directionalVar);
    double beta = std::sqrt(2) * erfcinv(2 * psafe_ - 1);
    return dist > obs.getRadius() + beta * sigma;
}

bool UncertaintyManager::isRectConstraintSatisfied(const Eigen::Vector2d& mean, 
    const Eigen::Matrix2d& cov, const Obstacle& obs) const
{
    Eigen::Vector2d obsCenter(obs.getX() + obs.getWidth()/2, obs.getY() + obs.getHeight()/2);
    double obsRadius = std::hypot(obs.getWidth()/2, obs.getHeight()/2);
    return isCircleConstraintSatisfied(mean, cov, Obstacle(obsCenter[0], obsCenter[1], obsRadius));
}

CCRRTMotionValidator::CCRRTMotionValidator(const ob::SpaceInformationPtr& si, double psafe)
    : ob::MotionValidator(si), uncertaintyManager_(psafe)
{
    // Cast to control::SpaceInformation to access control-specific methods
    auto csi = std::static_pointer_cast<oc::SpaceInformation>(si);
    dt_ = csi->getPropagationStepSize(); // Now valid
    // Initialize matrices after getting dt_
    int dim = si->getStateDimension();
    A_ = Eigen::MatrixXd::Identity(dim, dim);
    B_ = Eigen::MatrixXd::Identity(dim, 2);
    Pw_ = Eigen::MatrixXd::Identity(dim, dim) * 0.01;
}

void CCRRTMotionValidator::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

Eigen::VectorXd CCRRTMotionValidator::stateToVec(const ob::State* state) const {
    const auto* rs = state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::VectorXd vec(si_->getStateDimension());
    for (unsigned int i = 0; i < vec.size(); ++i)
        vec(i) = rs->values[i];
    return vec;
}

bool CCRRTMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const {
    if (!si_->isValid(s1) || !si_->isValid(s2)) return false;
    const unsigned int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
    std::vector<ob::State*> states(nd + 1);
    states[0] = si_->cloneState(s1);
    states[nd] = si_->cloneState(s2);
    // Use the getter method to access state uncertainty
    const auto& stateUncertainty = uncertaintyManager_.getStateUncertainty();
    auto s1_it = stateUncertainty.find(s1);
    auto s2_it = stateUncertainty.find(s2);
    // Get timestamps for start and end states
    double t1 = 0.0, t2 = 0.0;
    if (s1_it != stateUncertainty.end()) {
        t1 = s1_it->second.timestamp;
    }
    if (s2_it != stateUncertainty.end()) {
        t2 = s2_it->second.timestamp;
    }
    // Linear interpolation of timestamps
    const double dt = (t2 - t1) / nd;  
    for (unsigned int i = 1; i < nd; ++i) {
        states[i] = si_->allocState();
        si_->getStateSpace()->interpolate(s1, s2, (double)i / nd, states[i]);
        // Get current interpolated timestamp for dynamic obstacle position
        double currentTime = t1 + i * dt;
        // Update positions of dynamic obstacles in the validator
        std::vector<Obstacle> timeAdjustedObstacles = staticObstacles;
        for (const auto& dynObs : dynamicObstacles) {
            Eigen::Vector2d pos = dynObs.getPositionAtTime(currentTime);
            timeAdjustedObstacles.push_back(Obstacle(pos[0], pos[1], dynObs.getRadius() + 0.2)); // Inflate by 0.2 units
        }
        // Predict where dynamic obstacles will be when robot reaches this point
        double timeToReach = currentTime - t1;
        // Instead of redeclaring, clear and reuse the vector
        timeAdjustedObstacles.clear();
        timeAdjustedObstacles.insert(timeAdjustedObstacles.end(), staticObstacles.begin(), staticObstacles.end());
        for (const auto& dynObs : dynamicObstacles) {
            // Predict position with some lookahead
            Eigen::Vector2d pos = dynObs.getPositionAtTime(currentTime + 0.5*timeToReach);
            timeAdjustedObstacles.push_back(Obstacle(pos[0], pos[1], 
                                        dynObs.getRadius() + 0.3)); // Larger inflation
        }
        // Check state validity with current obstacle positions
        if (!uncertaintyManager_.satisfiesChanceConstraints(states[i], timeAdjustedObstacles)) {
            for (auto* s : states) si_->freeState(s);
            return false;
        }
    }
    // Propagate uncertainty and control
    for (unsigned int i = 1; i <= nd; ++i) {
        Eigen::VectorXd u = B_.inverse() * (stateToVec(states[i]) - A_ * stateToVec(states[i-1])) / dt_;
        uncertaintyManager_.propagateUncertainty(states[i-1], states[i], A_, B_, u, Pw_, dt_);
    }
    for (auto* s : states) si_->freeState(s);
    return true;
}

bool CCRRTMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2,
    std::pair<ob::State*, double>& lastValid) const
{
    if (!checkMotion(s1, s2)) {
        si_->copyState(lastValid.first, s1);
        lastValid.second = 0.0;
        return false;
    }
    return true;
}