///////////////////////////////////////
// RBE 550 - Motion Planning
// Project 4: RG-RRT Implementation
// Authors: Pranay Katyal, Harsh Chhajed
//////////////////////////////////////
#include "CollisionChecking.h"
#include <cmath>
#include <vector>

// Helper function to get car's rotated vertices
std::vector<std::pair<double, double>> getCarVertices(const Car& car) {
    double halfLength = car.length / 2.0;
    double halfWidth = car.width / 2.0;

    // Define car corners relative to its center
    std::vector<std::pair<double, double>> vertices = {
        {-halfLength, -halfWidth},
        {halfLength, -halfWidth},
        {halfLength, halfWidth},
        {-halfLength, halfWidth}
    };

    // Rotation matrix
    double cosTheta = std::cos(car.theta);
    double sinTheta = std::sin(car.theta);

    // Apply rotation and translation
    for (auto& vertex : vertices) {
        double x = vertex.first;
        double y = vertex.second;
        vertex.first = car.x + (x * cosTheta - y * sinTheta);
        vertex.second = car.y + (x * sinTheta + y * cosTheta);
    }

    return vertices;
}

// Check if a point is inside an obstacle
bool pointInObstacle(double x, double y, const Obstacle& obstacle) {
    return (x >= obstacle.x && x <= obstacle.x + obstacle.width &&
            y >= obstacle.y && y <= obstacle.y + obstacle.height);
}

// Collision checking implementation
bool checkCarObstacleCollision(const Car& car, const Obstacle& obstacle) {
    // Get car's rotated vertices
    auto vertices = getCarVertices(car);

    // Check if any car vertex is inside the obstacle
    for (const auto& vertex : vertices) {
        if (pointInObstacle(vertex.first, vertex.second, obstacle)) {
            return true;
        }
    }

    // Additional check: obstacle corners inside car
    std::vector<std::pair<double, double>> obstaclePoints = {
        {obstacle.x, obstacle.y},
        {obstacle.x + obstacle.width, obstacle.y},
        {obstacle.x, obstacle.y + obstacle.height},
        {obstacle.x + obstacle.width, obstacle.y + obstacle.height}
    };

    for (const auto& point : obstaclePoints) {
        // Complex point-in-polygon check using ray casting
        int intersections = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            if (((vertices[i].second > point.second) != (vertices[j].second > point.second)) &&
                (point.first < (vertices[j].first - vertices[i].first) * 
                 (point.second - vertices[i].second) / 
                 (vertices[j].second - vertices[i].second) + vertices[i].first)) {
                intersections++;
            }
        }
        if (intersections % 2 == 1) {
            return true;
        }
    }

    return false;
}

// Collision checking for entire car state
bool isCarStateValid(const Car& car, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (checkCarObstacleCollision(car, obstacle)) {
            return false;
        }
    }
    return true;
}

bool checkCollision(const Car& car, const Obstacle& obstacle) {
    // Check if the car collides with the obstacle
    return checkCarObstacleCollision(car, obstacle);
}