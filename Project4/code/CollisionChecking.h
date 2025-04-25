///////////////////////////////////////
// RBE 550 - Motion Planning
// Project 4: RG-RRT Implementation
// Authors: Pranay Katyal, Harsh Chhajed
//////////////////////////////////////
// In CollisionChecking.h
#ifndef COLLISION_CHECKING_H
#define COLLISION_CHECKING_H

#include <vector>
#include <utility>

// Separate struct for obstacles (axis-aligned)
struct Obstacle {
    double x;       // bottom-left x
    double y;       // bottom-left y
    double width;   // width of obstacle
    double height;  // height of obstacle
};

// Separate struct for car (can be rotated)
struct Car {
    double x;       // center x
    double y;       // center y
    double length;  // length of car
    double width;   // width of car
    double theta;   // rotation angle
};

// Function to check car collision with obstacles
bool checkCollision(const Car& car, const Obstacle& obstacle);

#endif // COLLISION_CHECKING_H