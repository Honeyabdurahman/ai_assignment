from queue import Queue, LifoQueue, PriorityQueue

# Define the graph
graph = {
    'S': {('A', 3), ('B', 1)},
    'A': {('B', 2), ('C', 2)},
    'B': {('C', 3)},
    'C': {('D', 4), ('G', 4)},
    'D': {('G', 1)},
    'G': set()
}

# Heuristic values for Greedy and A* Graph Search
heuristic_values = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Start and goal nodes for graph search
start_node = 'S'
goal_node = 'G'

# Function to get the path from the start node to the goal node
def get_path(parents, goal):
    path = [goal]
    while goal in parents:
        goal = parents[goal]
        path.insert(0, goal)
    return path

# a. Depth First Search (DFS)
def depth_first_search_graph(graph, start, goal):
    stack = LifoQueue()
    stack.put(start)
    visited = set()
    parents = {}

    while not stack.empty():
        node = stack.get()
        visited.add(node)

        if node == goal:
            return get_path(parents, goal)

        for neighbor, _ in sorted(graph.get(node, []), reverse=True):
            if neighbor not in visited:
                stack.put(neighbor)
                parents[neighbor] = node

    return None

# b. Breadth First Search (BFS)
def breadth_first_search_graph(graph, start, goal):
    queue = Queue()
    queue.put(start)
    visited = set()
    parents = {}

    while not queue.empty():
        node = queue.get()
        visited.add(node)

        if node == goal:
            return get_path(parents, goal)

        for neighbor, _ in sorted(graph.get(node, [])):
            if neighbor not in visited:
                queue.put(neighbor)
                parents[neighbor] = node

    return None

# c. Uniform Cost Search (UCS)
def uniform_cost_search_graph(graph, start, goal):
    priority_queue = PriorityQueue()
    priority_queue.put((0, start))
    visited = set()
    parents = {}
    costs = {node: float('inf') for node in graph}
    costs[start] = 0

    while not priority_queue.empty():
        cost, node = priority_queue.get()
        visited.add(node)

        if node == goal:
            return get_path(parents, goal)

        for neighbor, edge_cost in sorted(graph.get(node, []), key=lambda x: x[1]):
            if neighbor not in visited:
                new_cost = costs[node] + edge_cost
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    priority_queue.put((new_cost, neighbor))
                    parents[neighbor] = node

    return None

# d. Greedy Search
def greedy_search_graph(graph, start, goal, heuristic_values):
    priority_queue = PriorityQueue()
    priority_queue.put((heuristic_values[start], start))
    visited = set()
    parents = {}

    while not priority_queue.empty():
        _, node = priority_queue.get()
        visited.add(node)

        if node == goal:
            return get_path(parents, goal)

        for neighbor, _ in sorted(graph.get(node, []), key=lambda x: heuristic_values[x[0]]):
            if neighbor not in visited:
                priority_queue.put((heuristic_values[neighbor], neighbor))
                parents[neighbor] = node

    return None

# e. A* Search
def a_star_search_graph(graph, start, goal, heuristic_values):
    priority_queue = PriorityQueue()
    priority_queue.put((heuristic_values[start], 0, start))
    visited = set()
    parents = {}
    costs = {node: float('inf') for node in graph}
    costs[start] = 0

    while not priority_queue.empty():
        _, cost, node = priority_queue.get()
        visited.add(node)

        if node == goal:
            return get_path(parents, goal)

        for neighbor, edge_cost in sorted(graph.get(node, []), key=lambda x: x[1] + heuristic_values[x[0]]):
            if neighbor not in visited:
                new_cost = costs[node] + edge_cost
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    priority_queue.put((heuristic_values[neighbor] + new_cost, new_cost, neighbor))
                    parents[neighbor] = node

    return None

# Perform Graph Search for each strategy
dfs_graph_result = depth_first_search_graph(graph, start_node, goal_node)
bfs_graph_result = breadth_first_search_graph(graph, start_node, goal_node)
ucs_graph_result = uniform_cost_search_graph(graph, start_node, goal_node)
greedy_graph_result = greedy_search_graph(graph, start_node, goal_node, heuristic_values)
a_star_graph_result = a_star_search_graph(graph, start_node, goal_node, heuristic_values)

# Print results
print("Results:")
print("-" * 30)
print("\na. Depth First Search Graph Search:")
print("Path:", " -> ".join(dfs_graph_result) if dfs_graph_result else "No path found.")

print("\n b. Breadth First Search Graph Search:")
print("Path:", " -> ".join(bfs_graph_result) if bfs_graph_result else "No path found.")

print("\n c. Uniform Cost Search Graph Search:")
print("Path:", " -> ".join(ucs_graph_result) if ucs_graph_result else "No path found.")

print("\n d. Greedy Search Graph Search:")
print("Path:", " -> ".join(greedy_graph_result) if greedy_graph_result else "No path found.")

print("\n e. A* Search Graph Search:")
print("Path:", " -> ".join(a_star_graph_result) if a_star_graph_result else "No path found.")
