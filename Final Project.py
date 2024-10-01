import networkx as nx
import matplotlib.pyplot as plt
from queue import Queue


def dijkstra_shortest_path(graph, start, end):
    """
    Finds the shortest path and distance between two nodes
    in a graph using Dijkstra's algorithm.

    Parameters:
    - graph (dict): The graph represented as a dictionary.
    - start (str): The starting node.
    - end (str): The destination node.

    Returns:
    - Tuple: A tuple containing the shortest path and its distance.
    """
    G = nx.Graph(graph)
    shortest_path = nx.shortest_path(G, source=start, target=end,
                                     weight='weight')
    shortest_distance = nx.shortest_path_length(G, source=start, target=end,
                                                weight='weight')
    return shortest_path, shortest_distance


def dfs_traversal(graph, start):
    """
    Performs a Depth-First Search traversal on the graph.

    Parameters:
    - graph (dict): The graph represented as a dictionary.
    - start (str): The starting node for traversal.

    Returns:
    - List: The nodes traversed in DFS order.
    """
    visited = set()
    stack = [start]
    dfs_order = []

    while stack:
        current_node = stack.pop()
        if current_node not in visited:
            visited.add(current_node)
            dfs_order.append(current_node)
            stack.extend(neighbor for neighbor in graph[current_node])

    return dfs_order


def bfs_traversal(graph, start):
    """
    Performs a Breadth-First Search traversal on the graph.

    Parameters:
    - graph (dict): The graph represented as a dictionary.
    - start (str): The starting node for traversal.

    Returns:
    - List: The nodes traversed in BFS order.
    """
    visited = set()
    queue = Queue()
    bfs_order = []

    queue.put(start)
    visited.add(start)

    while not queue.empty():
        current_node = queue.get()
        bfs_order.append(current_node)

        for neighbor in graph[current_node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.put(neighbor)

    return bfs_order


def visualize_map(graph, start, end, edges_to_highlight,
                  title, save_path=None):
    """
    Visualizes a graph and highlights specific edges based on
    the traversal type.

    Parameters:
    - graph (dict): The graph represented as a dictionary.
    - start (str): The starting node for visualization.
    - end (str): The destination node for visualization.
    - edges_to_highlight (list): List of edges to highlight
    in the visualization.
    - title (str): The title of the visualization (e.g., 'Shortest Path',
    'DFS Traversal', 'BFS Traversal').

    Returns:
    - None: Displays the graph visualization.
    """
    G = nx.Graph(graph)

    # Define the positions of nodes
    pos = {
        'Shanghai Tower': (0, 0),
        'Nanjing Road': (1, 1),
        'The Bund': (2, 0),
        'Xintiandi': (3, 1),
        'Jingan Temple': (1, -1),
        'Yu Garden': (4, 0),
        'Pearl Tower': (5, 1)
    }

    # Draw the graph
    nx.draw(G, pos, with_labels=True, font_weight='bold',
            node_size=700, node_color='skyblue')

    # Highlight start and end locations
    nx.draw_networkx_nodes(G, pos, nodelist=[start, end], node_color='green',
                           node_size=700)

    # Display edge weights on the graph
    edge_labels = {(u, v): G[u][v].get('weight', '') for u, v in G.edges()}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels,
                                 font_color='red', font_size=8)

    # Highlight the specified edges
    nx.draw_networkx_edges(G, pos, edgelist=edges_to_highlight,
                           edge_color='purple' if title == 'Shortest Path'
                           else 'blue' if title == 'DFS Traversal' else 'red',
                           width=2)

    # Show the plot with the title
    plt.title(title)

    if save_path:
        plt.savefig(save_path, format="PNG")
    else:
        plt.show()


# Create a Shanghai Map
shanghai_map = {
    'Shanghai Tower': {'Nanjing Road': {'weight': 1},
                       'The Bund': {'weight': 3}, 'Xintiandi': {'weight': 2},
                       'Jingan Temple': {'weight': 4}},
    'Nanjing Road': {'Shanghai Tower': {'weight': 1},
                     'The Bund': {'weight': 2}, 'Xintiandi': {'weight': 4},
                     'Jingan Temple': {'weight': 3}},
    'The Bund': {'Shanghai Tower': {'weight': 3},
                 'Nanjing Road': {'weight': 2}, 'Xintiandi': {'weight': 3},
                 'Jingan Temple': {'weight': 5}},
    'Xintiandi': {'Shanghai Tower': {'weight': 2},
                  'Nanjing Road': {'weight': 4},
                  'The Bund': {'weight': 3}, 'Jingan Temple': {'weight': 1}},
    'Jingan Temple': {'Shanghai Tower': {'weight': 4},
                      'Nanjing Road': {'weight': 3},
                      'The Bund': {'weight': 5}, 'Yu Garden': {'weight': 8},
                      'Xintiandi': {'weight': 1}},
    'Yu Garden': {'Nanjing Road': {'weight': 6},
                  'Xintiandi': {'weight': 7},
                  'Jingan Temple': {'weight': 8},
                  'Pearl Tower': {'weight': 6}},
    'Pearl Tower': {'Xintiandi': {'weight': 8}, 'Yu Garden': {'weight': 6},
                    'Jingan Temple': {'weight': 9}, }
}

# Get user input for start and end locations
start_location = input("Enter your start location: ")
end_location = input("Enter your destination: ")

# Find and visualize the shortest path
shortest_path, shortest_distance = dijkstra_shortest_path(shanghai_map,
                                                          start_location,
                                                          end_location)

shortest_path_edges = [
    (shortest_path[i], shortest_path[i + 1])
    for i in range(len(shortest_path) - 1)
]
visualize_map(shanghai_map, start_location, end_location,
              shortest_path_edges, 'Shortest Path')

print(f"Shortest Path: {shortest_path}")
print(f"Shortest Distance: {shortest_distance} units")

# Perform and visualize DFS traversal
dfs_order = dfs_traversal(shanghai_map, start_location)
visualize_map(shanghai_map, start_location, end_location,
              [(dfs_order[i], dfs_order[i+1])
               for i in range(len(dfs_order)-1)], 'DFS Traversal')
print(f"DFS Traversal Order: {dfs_order}")

# Perform and visualize BFS traversal
bfs_order = bfs_traversal(shanghai_map, start_location)
visualize_map(shanghai_map, start_location, end_location,
              [(bfs_order[i], bfs_order[i+1])
               for i in range(len(bfs_order)-1)], 'BFS Traversal')
print(f"BFS Traversal Order: {bfs_order}")
