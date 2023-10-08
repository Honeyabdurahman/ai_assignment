# Define the graph
graph = {
    'S': {('A', 3), ('B', 1)},
    'A': {('B', 2), ('C', 2)},
    'B': {('C', 3)},
    'C': {('D', 4), ('G', 4)},
    'D': {('G', 1)},
    'G': set()
}

# Heuristic values for Greedy and A* Search
heuristic_values = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Start and goal nodes
start_node = 'S'
goal_node = 'G'

# a. Depth First Search (DFS)
def depth_first_search_tree(graph, start, goal):
    def dfs(node):
        if node == goal:
            return [node]

        for neighbor, cost in sorted(graph.get(node, []), key=lambda x: x[1], reverse=True):
            result = dfs(neighbor)
            if result:
                return [node] + result

        return None

    return dfs(start)

# b. Breadth First Search (BFS)
from collections import deque

def breadth_first_search_tree(graph, start, goal):
    queue = deque([(start, [])])

    while queue:
        node, path = queue.popleft()
        if node == goal:
            return path + [node]

        for neighbor, _ in sorted(graph.get(node, [])):
            queue.append((neighbor, path + [node]))

    return None

# c. Uniform Cost Search (UCS)
import heapq

def uniform_cost_search_tree(graph, start, goal):
    priority_queue = [(0, start, [])]

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        if node == goal:
            return path + [node]

        for neighbor, edge_cost in sorted(graph.get(node, []), key=lambda x: x[1]):
            heapq.heappush(priority_queue, (cost + edge_cost, neighbor, path + [node]))

    return None

# d. Greedy Search
def greedy_search_tree(graph, start, goal):
    def heuristic(node):
        return heuristic_values.get(node, float('inf'))

    priority_queue = [(heuristic(start), start, [])]

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        if node == goal:
            return path + [node]

        for neighbor, _ in sorted(graph.get(node, []), key=lambda x: heuristic(x[0])):
            heapq.heappush(priority_queue, (heuristic(neighbor), neighbor, path + [node]))

    return None

# e. A* Search
def a_star_search_tree(graph, start, goal):
    def heuristic(node):
        return heuristic_values.get(node, float('inf'))

    priority_queue = [(heuristic(start), 0, start, [])]

    while priority_queue:
        _, cost, node, path = heapq.heappop(priority_queue)
        if node == goal:
            return path + [node]

        for neighbor, edge_cost in sorted(graph.get(node, []), key=lambda x: x[1] + heuristic(x[0])):
            heapq.heappush(priority_queue, (cost + edge_cost + heuristic(neighbor), cost + edge_cost, neighbor, path + [node]))

    return None

# Performing Tree Search for each strategy
dfs_tree_result = depth_first_search_tree(graph, start_node, goal_node)
bfs_tree_result = breadth_first_search_tree(graph, start_node, goal_node)
ucs_tree_result = uniform_cost_search_tree(graph, start_node, goal_node)
greedy_tree_result = greedy_search_tree(graph, start_node, goal_node)
a_star_tree_result = a_star_search_tree(graph, start_node, goal_node)

# Print results
print("Results:")
print("-" * 30)

print("Path for Depth First Search - Tree search:")
print(" -> ".join(dfs_tree_result) if dfs_tree_result else "No path found.")

print("\nPath for Breadth First Search - Tree search:")
print(" -> ".join(bfs_tree_result) if bfs_tree_result else "No path found.")

print("\nPath for Uniform Cost Search - Tree search:")
print(" -> ".join(ucs_tree_result) if ucs_tree_result else "No path found.")

print("\nPath for Greedy Search - Tree search:")
print(" -> ".join(greedy_tree_result) if greedy_tree_result else "No path found.")

print("\nPath for A* Search - Tree search:")
print(" -> ".join(a_star_tree_result) if a_star_tree_result else "No path found.")
