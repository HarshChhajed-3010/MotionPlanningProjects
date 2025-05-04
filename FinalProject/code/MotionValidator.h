#ifndef MOTION_VALIDATOR_H
#define MOTION_VALIDATOR_H

#pragma once
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include "Obstacles.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Manages state uncertainty propagation and chance constraint checking
class UncertaintyManager {
public:
    // Initialize with desired safety probability threshold
    UncertaintyManager(double psafe) : psafe_(psafe) {}

    // Store uncertainty (mean and covariance) for a given state
    void storeUncertainty(const ob::State* state, 
        const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov, double timestamp);

    // Propagate uncertainty from one state to another using linear dynamics
    void propagateUncertainty(const ob::State* from, const ob::State* to,
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
        const Eigen::VectorXd& control, const Eigen::MatrixXd& Pw,
        double deltaTime);

    // Mark satisfiesChanceConstraints as const
    bool satisfiesChanceConstraints(const ob::State* state, 
        const std::vector<Obstacle>& obstacles) const;

    // Add timestamp to state uncertainty storage
    struct StateUncertainty {
        Eigen::VectorXd mean;
        Eigen::MatrixXd covariance;
        double timestamp;
    };

    // Add this getter method
    const std::map<const ob::State*, StateUncertainty>& getStateUncertainty() const {
        return stateUncertainty_;
    }

private:
    // Evaluate chance constraint for circular obstacles
    bool isCircleConstraintSatisfied(const Eigen::Vector2d& mean, 
        const Eigen::Matrix2d& cov, const Obstacle& obs) const;
    
    // Convert rectangular obstacles to circular for constraint checking
    bool isRectConstraintSatisfied(const Eigen::Vector2d& mean, 
        const Eigen::Matrix2d& cov, const Obstacle& obs) const;

    // Helper function to get dynamic obstacle position at time t
    Eigen::Vector2d getDynamicObstaclePosition(const Obstacle& obs, double time) const;

    double psafe_;  // Probability threshold for safety constraints
    std::map<const ob::State*, StateUncertainty> stateUncertainty_;
};

// Motion validator that incorporates uncertainty in collision checking
class CCRRTMotionValidator : public ob::MotionValidator {
public:
    // Initialize with space information and safety probability
    CCRRTMotionValidator(const ob::SpaceInformationPtr& si, double psafe);

    // Update obstacle list for collision checking
    void setObstacles(const std::vector<Obstacle>& obstacles);

    // Check if motion between states is valid considering uncertainty
    bool checkMotion(const ob::State* s1, const ob::State* s2) const override;

    // Check motion and return last valid state if motion is invalid
    bool checkMotion(const ob::State* s1, const ob::State* s2,
        std::pair<ob::State*, double>& lastValid) const override;

    // Access uncertainty manager for external use
    UncertaintyManager& getUncertaintyManager();

    Eigen::VectorXd stateToVec(const ob::State* state) const;

    Eigen::Vector2d getObstaclePosition(const Obstacle& obs) const;

    double getObstacleRadius(const Obstacle& obs) const;

    Obstacle createObstacle(const Eigen::Vector2d& pos, double radius) const;

private:
    mutable UncertaintyManager uncertaintyManager_;  // Manages state uncertainty
    std::vector<Obstacle> obstacles_;        // List of obstacles to check against
    double dt_;                       // Time step for state propagation
    Eigen::MatrixXd A_;                     // State transition matrix
    Eigen::MatrixXd B_;                     // Control input matrix
    Eigen::MatrixXd Pw_;                    // Process noise covariance
};

#endif // MOTION_VALIDATOR_H