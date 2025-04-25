#!/usr/bin/env python3
import os
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def read_obstacles(file_path):
    """
    Read the obstacles file.
    Each line should contain four numbers: x y width height.
    Returns a list of obstacles.
    """
    obstacles = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 4:
                continue
            x, y, w, h = map(float, parts[:4])
            obstacles.append((x, y, w, h))
    return obstacles

def read_path(file_path, system):
    """
    Read the solution path file.
    For the car: expect each line to have four numbers: x y theta v.
    For the pendulum: expect each line to have two numbers: theta omega.
    Returns a list of tuples.
    """
    path = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.split()
            if system == 'car':
                if len(parts) < 4:
                    continue
                x, y, theta, v = map(float, parts[:4])
                path.append((x, y, theta, v))
            elif system == 'pendulum':
                if len(parts) < 2:
                    continue
                theta, omega = map(float, parts[:2])
                path.append((theta, omega))
    return path

def visualize_car_path(obstacles, path, output_file):
    """
    Visualize the car solution path in the environment.
    Obstacles are drawn as rectangles.
    The path (x, y) is plotted as a blue line with markers.
    Start and goal states are annotated.
    """
    fig, ax = plt.subplots()

    # Draw each obstacle as a rectangle.
    for obs in obstacles:
        x, y, w, h = obs
        rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

    # Extract the x and y coordinates from the path.
    xs = [state[0] for state in path]
    ys = [state[1] for state in path]

    ax.plot(xs, ys, color='blue', label='Path')

    # Mark start and goal positions.
    if path:
        ax.plot(xs[0], ys[0], marker='s', color='green', markersize=10, label='Start')
        ax.plot(xs[-1], ys[-1], marker='*', color='red', markersize=10, label='Goal')
        

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Car Solution Path with Obstacles')
    ax.legend()
    plt.savefig(output_file, dpi=300)
    plt.close()

def visualize_pendulum_path(path, output_file):
    """
    Visualize the pendulum solution path in its phase space.
    The path is plotted as (theta, omega).
    Start and goal states are annotated.
    """
    fig, ax = plt.subplots()

    thetas = [state[0] for state in path]
    omegas = [state[1] for state in path]

    ax.plot(thetas, omegas, color='blue', label='Path')

    if path:
        ax.plot(thetas[0], omegas[0], marker='s', color='green', markersize=10, label='Start')
        ax.plot(thetas[-1], omegas[-1], marker='*', color='red', markersize=10, label='Goal 1')
        # Mirror goal across theta = 0 line
        #ax.plot(-thetas[-1], omegas[-1], marker='*', color='orange', markersize=10, linestyle='--', label=' Goal 2')

    ax.set_xlabel('Theta (rad)')
    ax.set_ylabel('Omega (rad/s)')
    ax.set_title('Pendulum Phase Space Path')
    ax.legend()
    plt.savefig(output_file, dpi=300)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Visualize solution paths for car or pendulum systems.")
    parser.add_argument("--system", type=str, choices=["car", "pendulum"], required=True,
                        help="System type: 'car' or 'pendulum'")
    parser.add_argument("--pathfile", type=str, required=True,
                        help="File containing the solution path")
    parser.add_argument("--obstacles", type=str,
                        help="File containing obstacles (required for car system)")
    parser.add_argument("--output", type=str, default="solution_path.png",
                        help="Output image file name")
    args = parser.parse_args()

    path = read_path(args.pathfile, args.system)
    if args.system == "car":
        if not args.obstacles:
            print("For car system, please provide an obstacles file using --obstacles")
            sys.exit(1)
        obstacles = read_obstacles(args.obstacles)
        visualize_car_path(obstacles, path, args.output)
    else:
        visualize_pendulum_path(path, args.output)

    print(f"Visualization saved as {args.output}")

if __name__ == "__main__":
    main()
