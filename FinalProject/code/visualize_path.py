#!/usr/bin/env python3
import os
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, FFMpegWriter

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

def read_tree(file_path):
    segs = []
    with open(file_path) as f:
        for line in f:
            x1, y1, x2, y2 = map(float, line.split())
            segs.append(((x1, y1), (x2, y2)))
    return segs

def animate_car_path(obstacles, path, covariances, save_video=False):
    fig, ax = plt.subplots()

    # Draw obstacles
    for x, y, w, h in obstacles:
        rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

    # Draw RRT tree
    for (x1, y1), (x2, y2) in read_tree("rrt_tree.txt"):
        ax.plot([x1, x2], [y1, y2], color='lightgray', linewidth=0.5, zorder=0)

    # Plot full path
    xs = [state[0] for state in path]
    ys = [state[1] for state in path]
    ax.plot(xs, ys, color='blue', label='Path')

    # Start and goal
    if path:
        ax.plot(xs[0], ys[0], marker='s', color='green', markersize=10, label='Start')
        ax.plot(xs[-1], ys[-1], marker='*', color='red', markersize=10, label='Goal')

    ax.set_aspect('equal', 'box')
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Animated Car Path')
    ax.legend()

    # Animation elements
    path_line, = ax.plot([], [], color='red', linewidth=2)

    car_length = 0.5
    car_width = 0.3
    car_patch = patches.Rectangle((0, 0), car_length, car_width, angle=0.0, color='red', zorder=5)
    ax.add_patch(car_patch)

    cov_ellipse = Ellipse((0, 0), 0, 0, edgecolor='orange', facecolor='none', lw=0.8, alpha=0.6)
    ax.add_patch(cov_ellipse)

    def init():
        path_line.set_data([], [])
        return path_line, car_patch, cov_ellipse

    def update(frame):
        x, y, theta, _ = path[frame]
        path_line.set_data(xs[:frame + 1], ys[:frame + 1])

        dx = -car_length / 2
        dy = -car_width / 2
        rot = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
        offset = rot @ np.array([dx, dy])
        car_patch.set_xy((x + offset[0], y + offset[1]))
        car_patch.angle = np.degrees(theta)

        cov = np.array(covariances[frame])
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        vals, vecs = vals[order], vecs[:, order]
        angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
        width, height = 2 * 1.96 * np.sqrt(vals)
        cov_ellipse.set_center((x, y))
        cov_ellipse.width = width
        cov_ellipse.height = height
        cov_ellipse.angle = angle

        return path_line, car_patch, cov_ellipse

    ani = FuncAnimation(fig, update, frames=len(path),
                        init_func=init, blit=True, interval=50, repeat=False)

    if save_video:
        print("[INFO] Saving animation to car_path_animation.mp4 ...")
        ani.save("car_path_animation.mp4", writer=FFMpegWriter(fps=20))
        print("[DONE] MP4 animation saved.")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Animate and save car path with uncertainty.")
    parser.add_argument("--pathfile", type=str, required=True, help="Path file with x y theta v per line")
    parser.add_argument("--obstacles", type=str, required=True, help="Obstacle file in x,y,w,h format per line")
    parser.add_argument("--savevideo", action="store_true", help="Save animation as MP4 instead of showing it")
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
    
    if len(covariances) < len(path):
        missing = len(path) - len(covariances)
        print(f"[WARN] Only {len(covariances)} covariances for {len(path)} path points; padding {missing} zeros.")
        zero_cov = ((0.0, 0.0), (0.0, 0.0))
        covariances.extend([zero_cov] * missing)

    animate_car_path(obstacles, path, covariances, save_video=args.savevideo)

if __name__ == "__main__":
    main()
