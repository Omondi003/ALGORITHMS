
# COMPARING DIKSTRA'S ALGORITHM AND BREADTH FIRST SEARCH

from collections import deque
import heapq
import numpy as np

# Constants to represent the grid
PASSABLE = 1
IMPASSABLE = float('inf')

# Directions for moving in the grid: up, down, left, right
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def is_valid_cell(grid, x, y):
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != IMPASSABLE

def bfs(grid, start, goal):
    queue = deque([start])
    visited = set([start])
    parent = {start: None}

    while queue:
        x, y = queue.popleft()

        if (x, y) == goal:
            return reconstruct_path(parent, start, goal)

        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if is_valid_cell(grid, nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                parent[(nx, ny)] = (x, y)
                queue.append((nx, ny))

    return None

def dijkstra(grid, start, goal):
    pq = [(0, start)]  # (cost, (x, y))
    visited = set()
    cost_so_far = {start: 0}
    parent = {start: None}

    while pq:
        current_cost, (x, y) = heapq.heappop(pq)

        if (x, y) in visited:
            continue
        visited.add((x, y))

        if (x, y) == goal:
            return reconstruct_path(parent, start, goal)

        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if is_valid_cell(grid, nx, ny):
                new_cost = current_cost + grid[nx][ny]
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    parent[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (new_cost, (nx, ny)))

    return None

def reconstruct_path(parent, start, goal):
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path

# Example Grid
grid = [
    [PASSABLE, PASSABLE, PASSABLE, IMPASSABLE],
    [IMPASSABLE, PASSABLE, PASSABLE, PASSABLE],
    [PASSABLE, PASSABLE, IMPASSABLE, PASSABLE],
    [PASSABLE, PASSABLE, PASSABLE, PASSABLE]
]

start = (0, 0)
goal = (3, 3)

# Run BFS
bfs_path = bfs(grid, start, goal)
print("BFS Path:", bfs_path)

# Run Dijkstra
dijkstra_path = dijkstra(grid, start, goal)
print("Dijkstra's Path:", dijkstra_path)
