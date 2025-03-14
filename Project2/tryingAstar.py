import heapq

# -----------------------------
# Heuristic Functions
# -----------------------------
def heuristic_admissible(node, goal):
    """
    Admissible heuristic: Chebyshev distance.
    Because moves can be diagonal, the minimum number of moves from node to goal is:
      h_a(x, y) = max(|x - g_x|, |y - g_y|)
    This heuristic never overestimates the true cost (when each move costs 1).
    """
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])
    return max(dx, dy)


def heuristic_nonadmissible(node, goal):
    """
    Non-admissible heuristic: Manhattan distance.
    In a grid with diagonal moves (each costing 1), the Manhattan distance:
      h_b(x, y) = |x - g_x| + |y - g_y|
    can overestimate the number of moves. For example, from (0,0) to (1,1) it returns 2
    even though one diagonal move (cost 1) suffices.
    """
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])
    return dx + dy


def heuristic_half_manhattan(node, goal):
    """
    A consistent (and admissible) heuristic: half the Manhattan distance.
    Since in any one move the Manhattan difference can change by at most 2, dividing by 2
    ensures that the difference in heuristic value between adjacent nodes is at most 1 (the move cost).
    """
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])
    return (dx + dy) / 2.0


def heuristic_combined(node, goal):
    """
    Combined heuristic: max{h1, h2} where:
      h1 = Chebyshev distance (heuristic_admissible)
      h2 = half Manhattan distance (heuristic_half_manhattan)
    This combined heuristic is consistent because for any two consistent heuristics h1 and h2,
    max{h1(n), h2(n)} is also consistent.
    """
    return max(heuristic_admissible(node, goal), heuristic_half_manhattan(node, goal))


# -----------------------------
# A* Search Implementation
# -----------------------------
def astar_search(start, goal, grid_width, grid_height, obstacles, heuristic):
    """
    A* search on a grid.
    
    Parameters:
      start: tuple (x, y) for start position.
      goal: tuple (x, y) for goal position.
      grid_width, grid_height: dimensions of the grid.
      obstacles: a set of (x, y) positions that cannot be traversed.
      heuristic: a function that takes (node, goal) and returns the heuristic cost.
      
    Returns:
      A tuple (path, cost, nodes_expanded) where:
        - path: a list of nodes from start to goal (if found).
        - cost: the total cost from start to goal.
        - nodes_expanded: total number of nodes expanded during the search.
    """
    # The open set is a priority queue of (f_score, node)
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    # For reconstructing the path
    came_from = {}
    # Cost from start to the current node
    g_score = {start: 0}
    
    nodes_expanded = 0

    while open_set:
        current_f, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, g_score[goal], nodes_expanded
        
        nodes_expanded += 1

        # Generate all valid neighbors (8 directions: vertical, horizontal, diagonal)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip the current node
                
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check if neighbor is within grid boundaries and not an obstacle.
                if 0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height:
                    if neighbor in obstacles:
                        continue
                    # For simplicity, each move has cost 1.
                    tentative_g = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))
                        came_from[neighbor] = current

    # If no path is found
    return None, float('inf'), nodes_expanded


# -----------------------------
# Main Simulation
# -----------------------------
def main():
    grid_width = 10
    grid_height = 10
    obstacles = set()  # No obstacles for simplicity. You can add (x, y) tuples if desired.
    
    start = (0, 0)
    goal = (7, 8)
    
    print("=== A* Search Simulation on a 10x10 Grid ===\n")
    
    # --- (a) Admissible vs. Non-Admissible Heuristics ---
    print("1. Using the admissible heuristic (Chebyshev distance):")
    path, cost, nodes = astar_search(start, goal, grid_width, grid_height, obstacles, heuristic_admissible)
    print("Path:", path)
    print("Cost:", cost)
    print("Nodes expanded:", nodes)
    print("\nNote: Chebyshev distance is admissible because it exactly counts the minimum number of moves required when diagonal moves are allowed.\n")
    
    print("2. Using the non-admissible heuristic (Manhattan distance):")
    path, cost, nodes = astar_search(start, goal, grid_width, grid_height, obstacles, heuristic_nonadmissible)
    print("Path:", path)
    print("Cost:", cost)
    print("Nodes expanded:", nodes)
    print("\nNote: Manhattan distance is non-admissible in this grid because, for example, from (0,0) to (1,1) it estimates a cost of 2 instead of 1.\n")
    
    # --- (b) Combining Consistent Heuristics ---
    print("3. Using a combined heuristic h(n) = max{h1(n), h2(n)} with:")
    print("   h1 = Chebyshev distance (admissible and consistent)")
    print("   h2 = half the Manhattan distance (consistent)")
    path, cost, nodes = astar_search(start, goal, grid_width, grid_height, obstacles, heuristic_combined)
    print("Path:", path)
    print("Cost:", cost)
    print("Nodes expanded:", nodes)
    print("\nNote: Since both h1 and h2 are consistent, the combined heuristic (their max) is also consistent.\n")
    

if __name__ == "__main__":
    main()
