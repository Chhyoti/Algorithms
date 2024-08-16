import heapq
import matplotlib.pyplot as plt

# Search Algorithms
def dfs(maze, start, goal):
    stack = [start]
    visited = set()
    parent = {start: None}

    while stack:
        current = stack.pop()
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        visited.add(current)
        for neighbor in get_neighbors(maze, current):
            if neighbor not in visited and neighbor not in stack:
                stack.append(neighbor)
                parent[neighbor] = current
    return None

def bfs(maze, start, goal):
    queue = [start]
    visited = set()
    parent = {start: None}

    while queue:
        current = queue.pop(0)
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        visited.add(current)
        for neighbor in get_neighbors(maze, current):
            if neighbor not in visited and neighbor not in queue:
                queue.append(neighbor)
                parent[neighbor] = current
    return None

def ucs(maze, start, goal):
    pq = [(0, start)]
    visited = set()
    parent = {start: None}
    cost = {start: 0}

    while pq:
        current_cost, current = heapq.heappop(pq)
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        visited.add(current)
        for neighbor in get_neighbors(maze, current):
            new_cost = current_cost + 1  # all edges have cost 1
            if neighbor not in visited or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                heapq.heappush(pq, (new_cost, neighbor))
                parent[neighbor] = current
    return None

def a_star(maze, start, goal):
    pq = [(0, start)]
    visited = set()
    parent = {start: None}
    cost = {start: 0}

    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        visited.add(current)
        for neighbor in get_neighbors(maze, current):
            new_cost = cost[current] + 1
            heuristic = manhattan_distance(neighbor, goal)
            if neighbor not in visited or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                heapq.heappush(pq, (new_cost + heuristic, neighbor))
                parent[neighbor] = current
    return None

def best_first_search(maze, start, goal):
    pq = [(manhattan_distance(start, goal), start)]
    visited = set()
    parent = {start: None}

    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        visited.add(current)
        for neighbor in get_neighbors(maze, current):
            if neighbor not in visited:
                heapq.heappush(pq, (manhattan_distance(neighbor, goal), neighbor))
                parent[neighbor] = current
    return None

# Helper Functions
def get_neighbors(maze, position):
    rows, cols = len(maze), len(maze[0])
    neighbors = []
    for r, c in [(position[0]-1, position[1]), (position[0]+1, position[1]),
                 (position[0], position[1]-1), (position[0], position[1]+1)]:
        if 0 <= r < rows and 0 <= c < cols and maze[r][c] == 0:
            neighbors.append((r, c))
    return neighbors

def reconstruct_path(parent, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent[current]
    path.append(start)
    path.reverse()
    return path

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Visualization
def visualize_maze(maze, path, start, goal):
    maze_copy = [row[:] for row in maze]
    for r, c in path:
        if (r, c) != start and (r, c) != goal:
            maze_copy[r][c] = 2

    plt.imshow(maze_copy, cmap='hot', interpolation='nearest')
    plt.show()

# Main Method
def main():
    maze = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start = (0, 0)
    goal = (4, 4)

    algorithms = {
        '1': ('DFS', dfs),
        '2': ('BFS', bfs),
        '3': ('UCS', ucs),
        '4': ('A*', a_star),
        '5': ('Best-First Search', best_first_search)
    }

    while True:
        print("\nChoose an algorithm:")
        for key, (name, _) in algorithms.items():
            print(f"{key}: {name}")
        choice = input("Algorithm: ")

        if choice in algorithms:
            name, algorithm = algorithms[choice]
            print(f"\nSelected algorithm: {name}")
            path = algorithm(maze, start, goal)
            if path:
                print("Path found:", path)
                visualize_maze(maze, path, start, goal)
            else:
                print("No path available")
        else:
            print("Invalid choice. Please select a valid algorithm.")

        again = input("Solve another maze? (y/n): ")
        if again.lower() != 'y':
            break

if __name__ == "__main__":
    main()
