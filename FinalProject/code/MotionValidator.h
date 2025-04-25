#pragma once
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Eigen/Dense>
#include <map>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Represents both circular and rectangular obstacles in the environment
struct Obstacle {
    // Constructor for circular obstacles
    Obstacle(const Eigen::Vector2d& c, double r) : center(c), radius(r), isCircular(true) {}
    
    // Constructor for rectangular obstacles
    Obstacle(double x_, double y_, double w, double h) : 
        x(x_), y(y_), width(w), height(h), isCircular(false) {}
    
    // Properties for circular obstacles
    Eigen::Vector2d center;
    double radius;
    
    // Properties for rectangular obstacles
    double x, y, width, height;
    
    // Flag to determine obstacle type
    bool isCircular;
};

// Manages state uncertainty propagation and chance constraint checking
class UncertaintyManager {
public:
    // Initialize with desired safety probability threshold
    UncertaintyManager(double psafe) : psafe_(psafe) {}

    // Store uncertainty (mean and covariance) for a given state
    void storeUncertainty(const ob::State* state, 
        const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

    // Propagate uncertainty from one state to another using linear dynamics
    void propagateUncertainty(const ob::State* from, const ob::State* to,
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
        const Eigen::VectorXd& control, const Eigen::MatrixXd& Pw);

    // Check if state satisfies chance constraints with all obstacles
    bool satisfiesChanceConstraints(const ob::State* state, 
        const std::vector<Obstacle>& obstacles);

private:
    // Evaluate chance constraint for circular obstacles
    bool isCircleConstraintSatisfied(const Eigen::Vector2d& mean, 
        const Eigen::Matrix2d& cov, const Obstacle& obs);
    
    // Convert rectangular obstacles to circular for constraint checking
    bool isRectConstraintSatisfied(const Eigen::Vector2d& mean, 
        const Eigen::Matrix2d& cov, const Obstacle& obs);

    double psafe_;  // Probability threshold for safety constraints
    std::map<const ob::State*, std::pair<Eigen::VectorXd, Eigen::MatrixXd>> stateUncertainty_;
};

// Motion validator that incorporates uncertainty in collision checking
class CCRRTMotionValidator : public ob::MotionValidator {
public:
    // Initialize with space information and safety probability
    CCRRTMotionValidator(const ob::SpaceInformationPtr& si, double psafe);

    // Update obstacle list for collision checking
    void setObstacles(const std::vector<Obstacle>& obstacles);

    // Check if motion between states is valid considering uncertainty
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2) const override;

    // Check motion and return last valid state if motion is invalid
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2,
        std::pair<ob::State*, double>& lastValid) const override;

    // Access uncertainty manager for external use
    UncertaintyManager& getUncertaintyManager() { return uncertaintyManager_; }

    Eigen::VectorXd stateToVec(const ob::State* state) const;
    // Convert state to Eigen vector for easier manipulation

    

private:
    UncertaintyManager uncertaintyManager_;  // Manages state uncertainty
    std::vector<Obstacle> obstacles_;        // List of obstacles to check against
    Eigen::MatrixXd A_;                     // State transition matrix
    Eigen::MatrixXd B_;                     // Control input matrix
    Eigen::MatrixXd Pw_;                    // Process noise covariance
};