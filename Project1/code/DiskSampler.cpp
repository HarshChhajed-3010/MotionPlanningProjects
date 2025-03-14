/* Author: Ali Golestaneh and Constantinos Chamzas and Pranay Katyal */
#include "DiskSampler.h"

bool isPointOutsideSquare(double x, double y, double theta, double s, double x_r, double y_r){
        // Translate the point relative to the square's center
        double x_t = x - x_r;
        double y_t = y - y_r;

        // Rotate the point back (inverse rotation)
        double x_prime = x_t * std::cos(theta) + y_t * std::sin(theta);
        double y_prime = -x_t * std::sin(theta) + y_t * std::cos(theta);

        // Check if the point is outside the square
        double half_side = s / 2.0;
        return !(std::abs(x_prime) <= half_side && std::abs(y_prime) <= half_side);
}


bool isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];

    // A square obstacle with and edge of size 2*sqrt(2) is located in location [-3,-2,] and rotated pi/4 degrees around its center.
    // Fill out this function that returns False when the state is inside/or the obstacle and True otherwise// 
 
    // ******* START OF YOUR CODE HERE *******//
    
    // Square parameters
    double square_size = 2 * std::sqrt(2); 
    double square_center_x = -3.0;
    double square_center_y = -2.0;
    double square_rotation = M_PI / 4.0;


    return isPointOutsideSquare(x , y, square_rotation, square_size, square_center_x, square_center_y);

    return true;
    // ******* END OF YOUR CODE HERE *******//
}


bool DiskSampler::sampleNaive(ob::State *state) 
{
    // ******* START OF YOUR CODE HERE *******//
    double sampleR = DiskSampler::rng_.uniformReal(0,10);
    double sampleTheta = DiskSampler::rng_.uniformReal(0,2*M_PI - std::numeric_limits<double>::epsilon());

    double x = sampleR * cos(sampleTheta);
    double y = sampleR * sin(sampleTheta);

    auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    r2state->values[0] = x;
    r2state->values[1] = y;

       
    // ******* END OF YOUR CODE HERE *******//
    
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}

bool DiskSampler::sampleCorrect(ob::State *state)
{
    // ******* START OF YOUR CODE HERE *******//
    double sampleR = sqrt(DiskSampler::rng_.uniformReal(0, 100));
    double sampleTheta = DiskSampler::rng_.uniformReal(0,2*M_PI - std::numeric_limits<double>::epsilon());

    double x = sampleR * cos(sampleTheta);
    double y = sampleR * sin(sampleTheta);

    auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    r2state->values[0] = x;
    r2state->values[1] = y;


    // ******* END OF YOUR CODE HERE *******//


    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}
