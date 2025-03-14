import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent  # Stores the parent node to backtrack the path

class RRT:
    def __init__(self, start, goal, max_distance=10, goal_bias=0.05, x_range=(0, 100), y_range=(0, 100)):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.max_distance = max_distance
        self.goal_bias = goal_bias
        self.x_range = x_range
        self.y_range = y_range
        self.nodes = [self.start]  # Tree starts with the initial node

    def get_random_point(self):
        """Sample a random point, sometimes biased towards the goal."""
        if random.random() < self.goal_bias:
            return (self.goal.x, self.goal.y)  # Bias toward goal
        else:
            return (random.uniform(*self.x_range), random.uniform(*self.y_range))

    def nearest_node(self, x, y):
        """Find the closest node in the tree."""
        return min(self.nodes, key=lambda node: np.hypot(node.x - x, node.y - y))

    def steer(self, from_node, to_x, to_y):
        """Move from 'from_node' toward (to_x, to_y) by at most max_distance."""
        dx, dy = to_x - from_node.x, to_y - from_node.y
        distance = np.hypot(dx, dy)
        if distance > self.max_distance:
            scale = self.max_distance / distance
            new_x, new_y = from_node.x + scale * dx, from_node.y + scale * dy
        else:
            new_x, new_y = to_x, to_y
        return Node(new_x, new_y, parent=from_node)

    def plan(self, max_iterations=1000):
        """Main RRT algorithm."""
        for _ in range(max_iterations):
            # Sample a random point
            rx, ry = self.get_random_point()
            
            # Find the nearest node
            nearest = self.nearest_node(rx, ry)
            
            # Extend towards the sampled point
            new_node = self.steer(nearest, rx, ry)

            # Assume all motions are valid (no obstacles in this simple version)
            self.nodes.append(new_node)

            # Check if goal is reached
            if np.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y) < self.max_distance:
                self.goal.parent = new_node
                self.nodes.append(self.goal)
                return self.get_path()  # Return the final path

        return None  # No path found

    def get_path(self):
        """Backtrack from goal to start to get the path."""
        path = []
        node = self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  # Reverse to get path from start to goal

    def draw(self):
        """Plot the tree and the found path."""
        plt.figure(figsize=(8, 8))
        for node in self.nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        if self.goal.parent:
            path = self.get_path()
            plt.plot(*zip(*path), "-r", linewidth=2, label="Path")

        plt.scatter([n.x for n in self.nodes], [n.y for n in self.nodes], c="blue", s=10)
        plt.scatter(*self.start.__dict__.values(), c="red", marker="o", label="Start")
        plt.scatter(*self.goal.__dict__.values(), c="black", marker="x", label="Goal")
        plt.legend()
        plt.show()

# Example usage
rrt = RRT(start=(10, 10), goal=(90, 90))
path = rrt.plan()
rrt.draw()
