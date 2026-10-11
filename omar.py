import heapq
import time

# Choose the map here: MAP_A, MAP_B, or MAP_C
MAP_A = [
    "S..#....",
    "##.#.##.",
    "....~~..",
    ".####.#.",
    "...~..#G"
]

MAP_B = [
    "S~~~~~~~G",
    ".#######.",
    ".........",
    ".#.#.#.#.",
    "........."
]

MAP_C = [
    "S..#....",
    ".#.#.##.",
    "...#..#.",
    ".###.##.",
    "....#..G"
]

GRID = MAP_A


def find_start_goal(grid):
    """Return the (row, column) positions of S and G."""
    start = None
    goal = None

    for r in range(len(grid)):
        for c in range(len(grid[r])):
            if grid[r][c] == "S":
                start = (r, c)
            elif grid[r][c] == "G":
                goal = (r, c)

    return start, goal


def get_neighbors(grid, cell):
    """Return walkable neighboring cells: up, down, left, right."""
    r, c = cell
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []

    for dr, dc in directions:
        nr = r + dr
        nc = c + dc

        if 0 <= nr < len(grid) and 0 <= nc < len(grid[nr]):
            if grid[nr][nc] != "#":
                neighbors.append((nr, nc))

    return neighbors


def move_cost(grid, cell):
    """Cost of entering a cell. S is never charged because we do not enter it."""
    r, c = cell
    if grid[r][c] == "~":
        return 3
    return 1


def manhattan(cell, goal):
    """Estimate the remaining cost using Manhattan distance."""
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])


def search(grid, algorithm):
    """Run UCS or A* and return path, cost, expanded nodes, frontier size, time."""
    start_time = time.perf_counter()
    start, goal = find_start_goal(grid)

    if start is None or goal is None:
        return None, None, 0, 0, time.perf_counter() - start_time

    # Heap entries: (priority, cost_so_far, cell)
    frontier = []
    first_priority = 0 if algorithm == "UCS" else manhattan(start, goal)
    heapq.heappush(frontier, (first_priority, 0, start))

    best_cost = {start: 0}
    parent = {}
    expanded = 0
    max_frontier = 1

    while frontier:
        priority, cost_so_far, current = heapq.heappop(frontier)

        # Ignore old heap entries that are no longer the cheapest known route.
        if cost_so_far != best_cost.get(current):
            continue

        if current == goal:
            path = []
            node = goal
            while node != start:
                path.append(node)
                node = parent[node]
            path.append(start)
            path.reverse()
            elapsed = time.perf_counter() - start_time
            return path, cost_so_far, expanded, max_frontier, elapsed

        expanded += 1

        for neighbor in get_neighbors(grid, current):
            new_cost = cost_so_far + move_cost(grid, neighbor)

            if new_cost < best_cost.get(neighbor, float("inf")):
                best_cost[neighbor] = new_cost
                parent[neighbor] = current

                if algorithm == "UCS":
                    new_priority = new_cost
                else:
                    new_priority = new_cost + manhattan(neighbor, goal)

                heapq.heappush(frontier, (new_priority, new_cost, neighbor))

        max_frontier = max(max_frontier, len(frontier))

    elapsed = time.perf_counter() - start_time
    return None, None, expanded, max_frontier, elapsed


def show_map(grid, path):
    """Print the map and mark path cells with * (keep S and G visible)."""
    if path is None:
        for row in grid:
            print(row)
        return

    path_cells = set(path)
    for r, row in enumerate(grid):
        shown = ""
        for c, symbol in enumerate(row):
            if (r, c) in path_cells and symbol not in ("S", "G"):
                shown += "*"
            else:
                shown += symbol
        print(shown)


def run_algorithm(grid, algorithm, map_name):
    path, cost, expanded, max_frontier, elapsed = search(grid, algorithm)

    print("\n" + "=" * 50)
    print("Map:", map_name, "| Algorithm:", algorithm)

    if path is None:
        print("No path found")
        print("Path length: N/A")
        print("Total path cost: N/A")
    else:
        print("Path:", path)
        print("Path length (moves):", len(path) - 1)
        print("Total path cost:", cost)

    print("Nodes expanded:", expanded)
    print("Maximum frontier size:", max_frontier)
    print("Run time (seconds):", f"{elapsed:.8f}")
    print("Map with path:")
    show_map(grid, path)

    return {
        "map": map_name,
        "algorithm": algorithm,
        "path_length": len(path) - 1 if path else None,
        "cost": cost,
        "expanded": expanded,
        "max_frontier": max_frontier,
        "time": elapsed,
        "path": path,
    }


def main():
    # Run both algorithms on all three required maps.
    maps = [("MAP_A", MAP_A), ("MAP_B", MAP_B), ("MAP_C", MAP_C)]
    results = []

    for map_name, grid in maps:
        for algorithm in ["UCS", "A*"]:
            results.append(run_algorithm(grid, algorithm, map_name))

    print("\n" + "=" * 50)
    print("RESULTS SUMMARY")
    print("Map | Algorithm | Length | Cost | Expanded | Max Frontier | Time (s)")
    for item in results:
        length = item["path_length"] if item["path_length"] is not None else "N/A"
        cost = item["cost"] if item["cost"] is not None else "N/A"
        print(
            f'{item["map"]} | {item["algorithm"]} | {length} | {cost} | '
            f'{item["expanded"]} | {item["max_frontier"]} | {item["time"]:.8f}'
        )


if __name__ == "__main__":
    main()