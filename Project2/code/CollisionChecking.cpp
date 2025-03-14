///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Thomas Herring,
// Date: 16/02/2025

/*** 
 * 
 * 1. (15 points) Fillout the missing functions in CollisionChecking.cpp by implement collision checking for a point robot within the plane, 
 * and a square robot with known side length that translates and rotates in the plane in. There are many ways to do collision checking with a 
 * robot that translates and rotates. Here are some things to consider 
• The obstacles will only be axis aligned rectangles.
• You can re-purpose the point inside the squre code from Project1.
• Using a line intersection algorithm might be helpful.
• Make sure that your collision checker accounts for all corner cases 
 * 
 * ***/
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>
#include <algorithm>

// Function to check if a point (x, y) is valid (i.e., not inside any obstacle)
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    // Iterate through each obstacle
    for (const auto& rect : obstacles)
    {
        // Check if the point is inside the current obstacle
        if (x >= rect.x && x <= rect.x + rect.width &&
            y >= rect.y && y <= rect.y + rect.height)
        {
            return false; // Point is inside an obstacle
        }
    }
    return true; // Point is outside all obstacles
}

// Function to check if a point lies on a line segment
bool onSegment(const std::pair<double, double>& p, const std::pair<double, double>& q, const std::pair<double, double>& r)
{
    // Check if q is within the bounding box of p and r
    if (q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) &&
        q.second <= std::max(p.second, r.second) && q.second >= std::min(p.second, r.second))
        return true;
    return false;
}

// Function to check if two line segments intersect
bool lineIntersectsLine(const std::pair<double, double>& p1, const std::pair<double, double>& p2,
                        const std::pair<double, double>& q1, const std::pair<double, double>& q2)
{
    // Helper function to determine the orientation of three points
    auto orientation = [](const std::pair<double, double>& a, const std::pair<double, double>& b, const std::pair<double, double>& c) {
        double val = (b.second - a.second) * (c.first - b.first) - (b.first - a.first) * (c.second - b.second);
        if (val == 0) return 0; // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    };

    // Calculate the orientation of each triplet of points
    int o1 = orientation(p1, p2, q1);
    int o2 = orientation(p1, p2, q2);
    int o3 = orientation(q1, q2, p1);
    int o4 = orientation(q1, q2, p2);

    // General case: if the orientations are different, the line segments intersect
    if (o1 != o2 && o3 != o4)
        return true;

    // Special cases: check if the points are collinear and lie on the segments
    if (o1 == 0 && onSegment(p1, q1, p2)) return true;
    if (o2 == 0 && onSegment(p1, q2, p2)) return true;
    if (o3 == 0 && onSegment(q1, p1, q2)) return true;
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;

    return false; // No intersection
}

// Function to check if a line segment intersects with a rectangle
bool lineIntersectsRect(const std::pair<double, double>& p1, const std::pair<double, double>& p2, const Rectangle& rect)
{
    // Check if the line intersects any of the four edges of the rectangle
    return lineIntersectsLine(p1, p2, {rect.x, rect.y}, {rect.x + rect.width, rect.y}) ||
           lineIntersectsLine(p1, p2, {rect.x, rect.y}, {rect.x, rect.y + rect.height}) ||
           lineIntersectsLine(p1, p2, {rect.x + rect.width, rect.y}, {rect.x + rect.width, rect.y + rect.height}) ||
           lineIntersectsLine(p1, p2, {rect.x, rect.y + rect.height}, {rect.x + rect.width, rect.y + rect.height});
}

// Function to check if a square robot is valid (i.e., not intersecting any obstacle and within bounds)
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles, double hbound, double lbound)
{
    // Calculate the four corners of the square based on its center (x, y), orientation (theta), and side length
    double halfSide = sideLength / 2.0;
    std::vector<std::pair<double, double>> corners(4);

    // Calculate the coordinates of each corner
    corners[0] = {x + halfSide * cos(theta) - halfSide * sin(theta), y + halfSide * sin(theta) + halfSide * cos(theta)};
    corners[1] = {x - halfSide * cos(theta) - halfSide * sin(theta), y - halfSide * sin(theta) + halfSide * cos(theta)};
    corners[2] = {x - halfSide * cos(theta) + halfSide * sin(theta), y - halfSide * sin(theta) - halfSide * cos(theta)};
    corners[3] = {x + halfSide * cos(theta) + halfSide * sin(theta), y + halfSide * sin(theta) - halfSide * cos(theta)};

    // Check if any corner is out of bounds
    for (const auto& corner : corners)
    {
        if (corner.first < 0 || corner.first > hbound || corner.second < 0 || corner.second > lbound)
        {
            return false; // Square is out of bounds
        }
    }

    // Check if any corner is inside an obstacle
    for (const auto& rect : obstacles)
    {
        for (const auto& corner : corners)
        {
            if (corner.first >= rect.x && corner.first <= rect.x + rect.width &&
                corner.second >= rect.y && corner.second <= rect.y + rect.height)
            {
                return false; // Square intersects with an obstacle
            }
        }
    }

    // Check if any edge of the square intersects with an obstacle
    for (size_t i = 0; i < corners.size(); ++i)
    {
        size_t next = (i + 1) % corners.size();
        for (const auto& rect : obstacles)
        {
            if (lineIntersectsRect(corners[i], corners[next], rect))
            {
                return false; // Square edge intersects with an obstacle
            }
        }
    }

    return true; // Square is outside all obstacles and within bounds
}