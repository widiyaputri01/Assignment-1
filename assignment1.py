import heapq
import math
import time

# Data posisi kota
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

# Data koneksi antar kota
roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

# Euclidean distance sebagai heuristic
def euclidean(coord1, coord2):
    return math.hypot(coord1[0] - coord2[0], coord1[1] - coord2[1])

# Jarak nyata antar dua kota
def cost(city1, city2):
    return euclidean(cities[city1], cities[city2])

# Fungsi membangun kembali rute dari came_from
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# A* Search
def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {city: float('inf') for city in cities}
    g_score[start] = 0
    f_score = {city: float('inf') for city in cities}
    f_score[start] = euclidean(cities[start], cities[goal])
    explored_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored_nodes += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored_nodes

        for neighbor in roads[current]:
            temp_g = g_score[current] + cost(current, neighbor)
            if temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score[neighbor] = temp_g + euclidean(cities[neighbor], cities[goal])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, explored_nodes

# GBFS
def greedy_bfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean(cities[start], cities[goal]), start))
    came_from = {}
    visited = set()
    explored_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored_nodes += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored_nodes

        visited.add(current)

        for neighbor in roads[current]:
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (euclidean(cities[neighbor], cities[goal]), neighbor))

    return None, explored_nodes

# Visualisasi rute
def print_path(path):
    print(" -> ".join(path))
    total = 0
    for i in range(len(path) - 1):
        d = cost(path[i], path[i+1])
        print(f"{path[i]} -> {path[i+1]} = {d:.2f}")
        total += d
    print(f"Total Distance: {total:.2f}")

# Perbandingan kedua algoritma
def compare_algorithms(start, goal):
    print(f"\n🚗 Comparing A* vs GBFS from {start} to {goal}...\n")

    # A*
    start_time = time.time()
    path_astar, nodes_astar = a_star(start, goal)
    time_astar = (time.time() - start_time) * 1000

    print(" A* Path:")
    if path_astar:
        print_path(path_astar)
    print(f"A* Time: {time_astar:.3f} ms")
    print(f"A* Nodes Explored: {nodes_astar}")

    # GBFS
    start_time = time.time()
    path_gbfs, nodes_gbfs = greedy_bfs(start, goal)
    time_gbfs = (time.time() - start_time) * 1000

    print("\n GBFS Path:")
    if path_gbfs:
        print_path(path_gbfs)
    print(f"GBFS Time: {time_gbfs:.3f} ms")
    print(f"GBFS Nodes Explored: {nodes_gbfs}")

# Jalankan
if __name__ == "__main__":
    compare_algorithms("A", "D")
