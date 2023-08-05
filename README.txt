Things to Note:
-disconnected graphs are allowed
- representation of a typical list is:
A A, (B, 3), so we have vertex label, label and end vertex and weight

Comments within functions explain more about the functions: Below are major classes: 

Edge Class:
The Edge class represents a connection between two vertices in the graph. It has two attributes: end_vertex and weight. The end_vertex attribute stores the name of the vertex that the edge connects to. The weight attribute stores the weight of the edge.

Vertex Class:
The Vertex class represents a single node in the graph. It has three attributes: name, visited, and edges. The name attribute stores the name of the vertex. The visited attribute is a boolean value that indicates whether the vertex has been visited during a graph traversal. The edges attribute is a list that stores the edges that the vertex connects to.

Graph Class:
The Graph class represents a graph data structure. It has one attribute: vertices. The vertices attribute is a dictionary that maps vertex names to their adjacency lists. The adjacency list for a vertex is a list of edges that the vertex connects to.

DFS Function:
The DFS function performs a depth-first search traversal of the graph starting from a specified vertex. It is implemented recursively using a DFS helper function. The DFS helper function takes a vertex object and a list of visited vertices as input parameters. The DFS algorithm works by visiting the current vertex and marking it as visited. It then recursively visits all of the unvisited neighbors of the current vertex. The DFS algorithm continues until all vertices in the graph have been visited.

BFS Function:
The BFS function performs a breadth-first search traversal of the graph starting from a specified vertex. It is implemented iteratively using a queue data structure. The BFS algorithm works by adding the starting vertex to the queue and marking it as visited. It then dequeues the next vertex from the queue and visits all of its unvisited neighbors. The BFS algorithm continues until all vertices in the graph have been visited.

Dijkstra Function:
The Dijkstra function finds the shortest path between two vertices in a weighted graph. It is implemented using Dijkstra's algorithm, which is a greedy algorithm that works by iteratively selecting the vertex with the shortest distance from the starting vertex and updating the distances of its neighbors. The Dijkstra function takes the graph, the starting vertex, and the target vertex as input parameters. It returns the shortest path from the starting vertex to the target vertex, as well as the total distance of the path.