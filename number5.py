from queue import Queue, PriorityQueue, LifoQueue

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

def get_path(parents, goal):
    path = [goal]
    while goal in parents:
        goal = parents[goal]
        path.insert(0, goal)
    return path

# a. Depth First Search (DFS)
def depth_first_search_tree(graph, start, goal):
    stack = LifoQueue()
    stack.put(start)
    visited = set()
    parents = {}
    expanded_states = []  # To record the order of expansion
    unexpanded_states = set()  # To record states that were not expanded

    while not stack.empty():
        node = stack.get()
        expanded_states.append(node)  # Record the order of expansion
        visited.add(node)

        if node == goal:
            path = get_path(parents, goal)
            return path, expanded_states, unexpanded_states

        neighbors = sorted(graph.get(node, []), reverse=True)
        for neighbor, _ in neighbors:
            if neighbor not in visited:
                stack.put(neighbor)
                parents[neighbor] = node
            else:
                unexpanded_states.add(neighbor)  # Record states that were not expanded

    return None, expanded_states, unexpanded_states

# b. Breadth First Search (BFS)
def breadth_first_search_tree(graph, start, goal):
    queue = Queue()
    queue.put(start)
    visited = set()
    parents = {}
    expanded_states = []  # To record the order of expansion
    unexpanded_states = set()  # To record states that were not expanded

    while not queue.empty():
        node = queue.get()
        expanded_states.append(node)  # Record the order of expansion
        visited.add(node)

        if node == goal:
            path = get_path(parents, goal)
            return path, expanded_states, unexpanded_states

        neighbors = sorted(graph.get(node, []))
        for neighbor, _ in neighbors:
            if neighbor not in visited:
                queue.put(neighbor)
                parents[neighbor] = node
            else:
                unexpanded_states.add(neighbor)  # Record states that were not expanded

    return None, expanded_states, unexpanded_states

# c. Uniform Cost Search (UCS)
def uniform_cost_search_tree(graph, start, goal):
    priority_queue = PriorityQueue()
    priority_queue.put((0, start))
    visited = set()
    parents = {}
    costs = {node: float('inf') for node in graph}
    costs[start] = 0
    expanded_states = []  # To record the order of expansion
    unexpanded_states = set()  # To record states that were not expanded

    while not priority_queue.empty():
        cost, node = priority_queue.get()
        expanded_states.append(node)  # Record the order of expansion
        visited.add(node)

        if node == goal:
            path = get_path(parents, goal)
            return path, expanded_states, unexpanded_states

        neighbors = sorted(graph.get(node, []), key=lambda x: x[1])
        for neighbor, edge_cost in neighbors:
            if neighbor not in visited:
                new_cost = costs[node] + edge_cost
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    priority_queue.put((new_cost, neighbor))
                    parents[neighbor] = node
            else:
                unexpanded_states.add(neighbor)  # Record states that were not expanded

    return None, expanded_states, unexpanded_states

# d. Greedy Search
def greedy_search_tree(graph, start, goal, heuristic_values):
    priority_queue = PriorityQueue()
    priority_queue.put((heuristic_values[start], start))
    visited = set()
    parents = {}
    expanded_states = []  # To record the order of expansion
    unexpanded_states = set()  # To record states that were not expanded

    while not priority_queue.empty():
        _, node = priority_queue.get()
        expanded_states.append(node)  # Record the order of expansion
        visited.add(node)

        if node == goal:
            path = get_path(parents, goal)
            return path, expanded_states, unexpanded_states

        neighbors = sorted(graph.get(node, []), key=lambda x: heuristic_values[x[0]])
        for neighbor, _ in neighbors:
            if neighbor not in visited:
                priority_queue.put((heuristic_values[neighbor], neighbor))
                parents[neighbor] = node
            else:
                unexpanded_states.add(neighbor)  # Record states that were not expanded

    return None, expanded_states, unexpanded_states

# e. A* Search
def a_star_search_tree(graph, start, goal, heuristic_values):
    priority_queue = PriorityQueue()
    priority_queue.put((heuristic_values[start], 0, start))
    visited = set()
    parents = {}
    costs = {node: float('inf') for node in graph}
    costs[start] = 0
    expanded_states = []  # To record the order of expansion
    unexpanded_states = set()  # To record states that were not expanded

    while not priority_queue.empty():
        _, cost, node = priority_queue.get()
        expanded_states.append(node)  # Record the order of expansion
        visited.add(node)

        if node == goal:
            path = get_path(parents, goal)
            return path, expanded_states, unexpanded_states

        neighbors = sorted(graph.get(node, []), key=lambda x: x[1] + heuristic_values[x[0]])
        for neighbor, edge_cost in neighbors:
            if neighbor not in visited:
                new_cost = costs[node] + edge_cost
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    priority_queue.put((heuristic_values[neighbor] + new_cost, new_cost, neighbor))
                    parents[neighbor] = node
            else:
                unexpanded_states.add(neighbor)  # Record states that were not expanded

    return None, expanded_states, unexpanded_states

# ...

# Perform Tree Search for each strategy
dfs_tree_result, dfs_expanded, dfs_unexpanded = depth_first_search_tree(graph, start_node, goal_node)
bfs_tree_result, bfs_expanded, bfs_unexpanded = breadth_first_search_tree(graph, start_node, goal_node)
ucs_tree_result, ucs_expanded, ucs_unexpanded = uniform_cost_search_tree(graph, start_node, goal_node)
greedy_tree_result, greedy_expanded, greedy_unexpanded = greedy_search_tree(graph, start_node, goal_node, heuristic_values)
a_star_tree_result, a_star_expanded, a_star_unexpanded = a_star_search_tree(graph, start_node, goal_node, heuristic_values)

# Print results
print("Results:")
print("-" * 30)

print("\na. Depth First Tree Search (DFS)\n")
print("Expanded States Order:", " -> ".join(dfs_expanded))
print("Path:", " -> ".join(dfs_tree_result) if dfs_tree_result else "No path found.")
print("Unexpanded States:", "[{}]".format(" -> ".join(dfs_unexpanded)) if dfs_unexpanded else "No unexpanded states.")

print("\nb. Breadth First Tree Search (BFS)\n")
print("Expanded States Order:", " -> ".join(bfs_expanded))
print("Path:", " -> ".join(bfs_tree_result) if bfs_tree_result else "No path found.")
print("Unexpanded States:", "[{}]".format(" -> ".join(bfs_unexpanded)) if bfs_unexpanded else "No unexpanded states.")

print("\nc. Uniform Cost Tree Search (UCS)\n")
print("Expanded States Order:", " -> ".join(ucs_expanded))
print("Path:", " -> ".join(ucs_tree_result) if ucs_tree_result else "No path found.")
print("Unexpanded States:", "[{}]".format(" -> ".join(ucs_unexpanded)) if ucs_unexpanded else "No unexpanded states.")

print("\nd. Greedy Tree Search\n")
print("Expanded States Order:", " -> ".join(greedy_expanded))
print("Path:", " -> ".join(greedy_tree_result) if greedy_tree_result else "No path found.")
print("Unexpanded States:", "[{}]".format(" -> ".join(greedy_unexpanded)) if greedy_unexpanded else "No unexpanded states.")

print("\ne. A* Tree Search\n")
print("Expanded States Order:", " -> ".join(a_star_expanded))
print("Path:", " -> ".join(a_star_tree_result) if a_star_tree_result else "No path found.")
print("Unexpanded States:", "[{}]".format(" -> ".join(a_star_unexpanded)) if a_star_unexpanded else "No unexpanded states.")
