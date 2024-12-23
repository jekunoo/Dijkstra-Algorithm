import heapq
from tabulate import tabulate
import matplotlib.pyplot as plt
import networkx as nx
from termcolor import colored

def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {node: None for node in graph}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, previous_nodes

def shortest_path(previous_nodes, start, target):
    path = []
    current_node = target

    while current_node != start:
        if current_node is None:
            return []  # No path found
        path.append(current_node)
        current_node = previous_nodes[current_node]

    path.append(start)
    path.reverse()
    return path

def visualize_graph(graph, shortest_path=None):
    G = nx.DiGraph()
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            G.add_edge(node, neighbor, weight=weight)

    pos = nx.spring_layout(G)
    edge_labels = nx.get_edge_attributes(G, 'weight')

    plt.figure(figsize=(10, 7))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=12)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')

    if shortest_path:
        path_edges = list(zip(shortest_path, shortest_path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='blue', width=2.5)

    plt.title("Graph Visualization with Dijkstra's Shortest Path")
    plt.show()

def generate_table(graph, start, path, previous_nodes):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    visited = set()  # Use set for fast lookups
    table_data = []
    step = 1  # Step counter to label each iteration

    while priority_queue:
        # Ensure that "Step X" is printed only once per iteration
        print(f"Step {step}:")
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue

        visited.add(current_node)  # Mark the node as visited
        row = {}

        # Highlight the current node being processed in blue
        for node in graph:
            if distances[node] == float('inf'):
                row[node] = '99'
            else:
                previous = previous_nodes[node]
                if previous:
                    value = f"{int(distances[node])}/{previous}"
                    if node == current_node:  # Highlight the current node in blue
                        row[node] = colored(value, 'blue', attrs=['bold'])
                    else:
                        row[node] = value
                else:
                    value = f"{int(distances[node])}/{start}" if node == start else '99'
                    row[node] = value

        row['Visited'] = current_node
        table_data.append(row)

        # Print the table at each iteration
        nodes = list(graph.keys())
        headers = ['Visited'] + nodes
        rows = []
        for row in table_data:
            rows.append([row['Visited']] + [row[node] for node in nodes])

        print(tabulate(rows, headers=headers, tablefmt="grid"))
        print("\n")  # Blank line for better readability between iterations

        # Update distances for neighbors and push them to the priority queue
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

        step += 1  # Increment step after each iteration

    return distances, previous_nodes
# Example graph representation
# The graph is represented as a dictionary where each node points to its neighbors and edge weights.
graph = {
    'A': {'B': 6, 'D': 12},
    'B': {'C': 5},
    'C': {'D': 6,'E': 15, 'F': 14 },
    'D': {'E': 8,'F':7},
    'E': {'G': 4},
    'F': {'E': 2, 'I': 10},
    'G': {'I': 6},
    'I': {}
}

start_node = 'A'
target_node = 'I'

# Run Dijkstra's algorithm
distances, previous_nodes = dijkstra(graph, start_node)

# Find the shortest path
path = shortest_path(previous_nodes, start_node, target_node)

# Generate and display the progress table
generate_table(graph, start_node, path, previous_nodes)

# Print the results
print(f"Shortest distances from {start_node}: {distances}")
print(f"Shortest path from {start_node} to {target_node}: {path}")

# Visualize the graph and the shortest path
visualize_graph(graph, shortest_path=path)