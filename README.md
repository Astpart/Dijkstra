# Dijkstra's Algorithm for Finding Shortest Paths in a Graph

This Python script implements Dijkstra's algorithm for finding the shortest paths from a source vertex to all other vertices in a graph. Dijkstra's algorithm is efficient for graphs with non-negative edge weights.

## Description

Dijkstra's algorithm initializes distances from the source vertex to all other vertices as infinity and then iteratively relaxes these distances by considering the shortest path tree set. It selects the vertex with the minimum distance from the source vertex that is not yet included in the shortest path tree set and updates the distances of its adjacent vertices.

## CustomGraph Class

The `CustomGraph` class uses an adjacency matrix representation of the graph and provides methods to compute shortest paths using Dijkstra's algorithm and display distances from the source vertex.

### Methods

- `__init__(self, num_vertices)`: Initializes a graph instance with a specified number of vertices.
- `display_distances(self, distances)`: Prints the distances of all vertices from the source vertex.
- `find_min_distance(self, distances, shortest_path_tree_set)`: Finds the vertex with the minimum distance from the source vertex that is not yet included in the shortest path tree set.
- `dijkstra_algorithm(self, source)`: Computes the shortest paths from the `source` vertex using Dijkstra's algorithm.

### Example Usage

```python
class CustomGraph:

    def __init__(self, num_vertices):
        self.num_vertices = num_vertices
        self.graph = [[0 for _ in range(num_vertices)] for _ in range(num_vertices)]

    def display_distances(self, distances):
        print("Vertex \t Distance from Source")
        for vertex in range(self.num_vertices):
            print(vertex, "\t\t", distances[vertex])

    def find_min_distance(self, distances, shortest_path_tree_set):
        min_distance = float("Inf")
        min_index = -1

        for vertex in range(self.num_vertices):
            if distances[vertex] < min_distance and not shortest_path_tree_set[vertex]:
                min_distance = distances[vertex]
                min_index = vertex

        return min_index

    def dijkstra_algorithm(self, source):
        distances = [float("Inf")] * self.num_vertices
        distances[source] = 0
        shortest_path_tree_set = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            u = self.find_min_distance(distances, shortest_path_tree_set)
            shortest_path_tree_set[u] = True

            for v in range(self.num_vertices):
                if (self.graph[u][v] > 0 and not shortest_path_tree_set[v] and
                        distances[v] > distances[u] + self.graph[u][v]):
                    distances[v] = distances[u] + self.graph[u][v]

        self.display_distances(distances)

# Create a graph instance and define its adjacency matrix
my_graph = CustomGraph(9)
my_graph.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
                  [4, 0, 8, 0, 0, 0, 0, 11, 0],
                  [0, 8, 0, 7, 0, 4, 0, 0, 2],
                  [0, 0, 7, 0, 9, 14, 0, 0, 0],
                  [0, 0, 0, 9, 0, 10, 0, 0, 0],
                  [0, 0, 4, 14, 10, 0, 2, 0, 0],
                  [0, 0, 0, 0, 0, 2, 0, 1, 6],
                  [8, 11, 0, 0, 0, 0, 1, 0, 7],
                  [0, 0, 2, 0, 0, 0, 6, 7, 0]]

# Compute shortest paths from vertex 0 using Dijkstra's algorithm
my_graph.dijkstra_algorithm(0)
