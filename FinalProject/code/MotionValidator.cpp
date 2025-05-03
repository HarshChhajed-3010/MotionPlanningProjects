// MotionValidator.cpp
#include "MotionValidator.h"
#include <ompl/control/SpaceInformation.h>
#include <cmath>

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
    const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov) 
{
    stateUncertainty_[state] = std::make_pair(mean, cov);
}

void UncertaintyManager::propagateUncertainty(const ob::State* from, const ob::State* to,
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::VectorXd& control, const Eigen::MatrixXd& Pw)
{
    auto it = stateUncertainty_.find(from);
    if (it == stateUncertainty_.end()) {
        // Initialize with process noise (Pw) instead of a fixed small diagonal
        int dim = A.rows(); // Use actual state dimension from matrix A
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(dim);
        const auto* rs = from->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < dim; ++i)
            mean(i) = rs->values[i];
        stateUncertainty_[from] = std::make_pair(mean, Pw); // Use Pw here
        it = stateUncertainty_.find(from);
    }

    // Propagate mean and covariance
    Eigen::VectorXd nextMean = A * it->second.first + B * control;
    Eigen::MatrixXd nextCov = A * it->second.second * A.transpose() + Pw;
    stateUncertainty_[to] = std::make_pair(nextMean, nextCov);
}

bool UncertaintyManager::satisfiesChanceConstraints(const ob::State* state, 
    const std::vector<Obstacle>& obstacles) const
{
    auto it = stateUncertainty_.find(state);
    if (it == stateUncertainty_.end()) return true;

    Eigen::Vector2d mean = it->second.first.head<2>();
    Eigen::Matrix2d cov = it->second.second.block<2,2>(0, 0);

    for (const auto& obs : obstacles) {
        if (obs.isCircular && !isCircleConstraintSatisfied(mean, cov, obs)) return false;
        if (!obs.isCircular && !isRectConstraintSatisfied(mean, cov, obs)) return false;
    }
    return true;
}

bool UncertaintyManager::isCircleConstraintSatisfied(const Eigen::Vector2d& mean, 
    const Eigen::Matrix2d& cov, const Obstacle& obs) const
{
    Eigen::Vector2d diff = obs.center - mean;
    double dist = diff.norm();
    
    double directionalVar = (dist > 1e-6) ? 
        (diff.normalized().transpose() * cov * diff.normalized()) :
        cov.eigenvalues().real().maxCoeff();
    
    double sigma = std::sqrt(directionalVar);
    double beta = std::sqrt(2) * erfcinv(2 * psafe_ - 1);
    return dist > obs.radius + beta * sigma;
}

bool UncertaintyManager::isRectConstraintSatisfied(const Eigen::Vector2d& mean, 
    const Eigen::Matrix2d& cov, const Obstacle& obs) const
{
    Eigen::Vector2d obsCenter(obs.x + obs.width/2, obs.y + obs.height/2);
    double obsRadius = std::hypot(obs.width/2, obs.height/2);
    return isCircleConstraintSatisfied(mean, cov, Obstacle(obsCenter, obsRadius));
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

    for (unsigned int i = 1; i < nd; ++i) {
        states[i] = si_->allocState();
        si_->getStateSpace()->interpolate(s1, s2, (double)i / nd, states[i]);
    }

    for (unsigned int i = 1; i <= nd; ++i) {
        Eigen::VectorXd u = B_.inverse() * (stateToVec(states[i]) - A_ * stateToVec(states[i - 1])) / dt_;
        
        // Propagate uncertainty (allowed because uncertaintyManager_ is mutable)
        uncertaintyManager_.propagateUncertainty(states[i - 1], states[i], A_, B_, u, Pw_);
        
        if (!uncertaintyManager_.satisfiesChanceConstraints(states[i], obstacles_)) {
            for (auto* s : states) si_->freeState(s);
            return false;
        }
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