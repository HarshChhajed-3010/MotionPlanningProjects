#!/usr/bin/env python3
import os
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.patches import Ellipse

def read_obstacles(file_path):
    obstacles = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 4:
                continue
            x, y, w, h = map(float, parts[:4])
            obstacles.append((x, y, w, h))
    return obstacles

def read_path(file_path):
    path = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.split()
            if len(parts) < 4:
                continue
            x, y, theta, v = map(float, parts[:4])
            path.append((x, y, theta, v))
    return path

def read_covariances(file_path):
    covariances = []
    with open(file_path, 'r') as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) == 4:
                covariances.append(((vals[0], vals[1]), (vals[2], vals[3])))
    return covariances

def plot_cov_ellipse(ax, mean, cov, n_std=1.96, **kwargs):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=theta, **kwargs)
    ax.add_patch(ellipse)

def read_tree(file_path):
    segs = []
    with open(file_path) as f:
        for line in f:
            x1,y1,x2,y2 = map(float, line.split())
            segs.append(((x1,y1),(x2,y2)))
    return segs

def visualize_car_path(obstacles, path, covariances, output_file):
    fig, ax = plt.subplots()

    # Draw obstacles
    for x, y, w, h in obstacles:
        rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

    # Plot path
    xs = [state[0] for state in path]
    ys = [state[1] for state in path]
    ax.plot(xs, ys, color='blue', label='Path')

    # Draw RRT tree segments
    for (x1,y1),(x2,y2) in read_tree("rrt_tree.txt"):
        ax.plot([x1,x2],[y1,y2], color='lightgray', linewidth=0.5, zorder=0)

    # Annotate start and goal
    if path:
        ax.plot(xs[0], ys[0], marker='s', color='green', markersize=10, label='Start')
        ax.plot(xs[-1], ys[-1], marker='*', color='red', markersize=10, label='Goal')

    # Plot covariance ellipses for every point
    for i, (x, y, _, _) in enumerate(path):
        cov = covariances[i]
        C = np.array(cov)
        plot_cov_ellipse(ax, (x, y), C, edgecolor='orange', facecolor='none', lw=0.8, alpha=0.6)

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Car Solution Path with Obstacles')
    ax.legend()
    plt.savefig(output_file, dpi=300)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Visualize solution path for a car system.")
    parser.add_argument("--pathfile", type=str, required=True, help="Path file with x y theta v per line")
    parser.add_argument("--obstacles", type=str, required=True, help="Obstacle file in x,y,w,h format per line")
    parser.add_argument("--output", type=str, default="solution_path.png", help="Output image filename")
    args = parser.parse_args()

    path = read_path(args.pathfile)
    obstacles = read_obstacles(args.obstacles)

    cov_file = "covariances.txt"
    covariances = []
    if os.path.exists(cov_file):
        covariances = read_covariances(cov_file)
        print(f"[INFO] Loaded {len(covariances)} covariance entries from '{cov_file}'")
    else:
        print(f"[INFO] No covariance file found ('{cov_file}'). Ellipses will be zero-sized.")

    # Pad covariance list to match path length
    if len(covariances) < len(path):
        missing = len(path) - len(covariances)
        print(f"[WARN] Only {len(covariances)} covariances for {len(path)} path points; padding {missing} zeros.")
        zero_cov = ((0.0, 0.0), (0.0, 0.0))
        covariances.extend([zero_cov] * missing)

    visualize_car_path(obstacles, path, covariances, args.output)
    print(f"Visualization saved as {args.output}")

if __name__ == "__main__":
    main()
