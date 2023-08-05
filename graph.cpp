#include <queue>
#include <climits>
#include <set>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>

#include "graph.h"

/**
 * A graph is made up of vertices and edges
 * A vertex can be connected to other vertices via weighted, directed edge
 */


 ////////////////////////////////////////////////////////////////////////////////
 // This is 80 characters - Keep all lines under 80 characters                 //
 ////////////////////////////////////////////////////////////////////////////////


 /** constructor, empty graph */
Graph::Graph() {}

/** destructor, delete all vertices and edges
    only vertices stored in map
    no pointers to edges created by graph */
Graph::~Graph() {
    for (auto &it : vertices) {
        delete it.second;
    }
    vertices.clear();
}

/** return number of vertices */
int Graph::getNumVertices() const {
    return this->numberOfVertices;
}

/** return number of vertices */
int Graph::getNumEdges() const {
    return this->numberOfEdges;
}

/** add a new edge between start and end vertex
    if the vertices do not exist, create them
    calls Vertex::connect
    a vertex cannot connect to itself
    or have multiple edges to another vertex */
bool Graph::add(std::string start, std::string end, int edgeWeight) {
    if (start == end) {
        return false;
    }
    Vertex* startVertex = findOrCreateVertex(start);
    Vertex* endVertex = findOrCreateVertex(end);
    if (startVertex == endVertex) {
        return false;
    }
    startVertex->connect(end, edgeWeight);
    numberOfEdges++;
    return true;
}

/** return weight of the edge between start and end
    returns INT_MAX if not connected or vertices don't exist */
int Graph::getEdgeWeight(std::string start, std::string end) const {
    auto startVertex = vertices.find(start);
    if (startVertex == vertices.end()) {// if iterator reaches end
        //doesnt exists two conditions met
        return INT_MAX;
    }
    int weight = startVertex->second->getEdgeWeight(end);
    //did a arrow here because i am alluding to the vertex
    //which is the second parameter in vertices
    // to the get edge function within vertex class
    if (weight == -1) {// basically what this does i verify
        //that no edge exists sicne every edge has weight
        return INT_MAX;
    }
    return weight;
}

/** read edges from file
    the first line of the file is an integer, indicating number of edges
    each edge line is in the form of "string string int"
    fromVertex  toVertex    edgeWeight */
void Graph::readFile(std::string filename) {
    ifstream inputFile(filename);
    if (!inputFile) {
        cout << "Unable to open file : " << filename << endl;
        return;
    }
    int numEdges = 0;
    inputFile >> numEdges;
    for (int i = 0; i < numEdges; i++) {
        string fromVertex, toVertex;
        int edgeWeight;
        inputFile >> fromVertex >> toVertex >> edgeWeight;
        // predefined add function
        add(fromVertex, toVertex, edgeWeight);
    }
    inputFile.close();
}

/** depth-first traversal starting from startLabel
    call the function visit on each vertex label */
void Graph::depthFirstTraversal(std::string startLabel,
    void visit(const std::string&)) {
    for (auto& it : vertices) {
        it.second->unvisit();
    }
    Vertex* startVertex = vertices[startLabel];
    depthFirstTraversalHelper(startVertex, visit);
}

/** breadth-first traversal starting from startLabel
    call the function visit on each vertex label */
void Graph::breadthFirstTraversal(std::string startLabel,
    void visit(const std::string&)) {
    for (auto &it : vertices) {
        it.second->unvisit();
    }
    queue<Vertex*> vertexQueue;
    Vertex* startVertex = vertices[startLabel];
    if (startVertex == nullptr) {
        cout << "nodes dont exist";
        return;
    }
    vertexQueue.push(startVertex);
    while (!vertexQueue.empty()) {
        Vertex* tempVertex = vertexQueue.front();
        vertexQueue.pop();
        if (tempVertex != nullptr) {
            visit(tempVertex->getLabel());
            tempVertex->visit();
            tempVertex->resetNeighbor();
            string temp = tempVertex->getNextNeighbor();
            while (temp != tempVertex->getLabel()) {
                Vertex* neighborVertex = vertices[temp];
                if (!neighborVertex->isVisited()
                    && neighborVertex != nullptr) {
                    neighborVertex->visit();
                    vertexQueue.push(neighborVertex);
                }
                temp = tempVertex->getNextNeighbor();
            }
        }
    }
}

/** find the lowest cost from startLabel to all vertices that can be reached
    using Djikstra's shortest-path algorithm
    record costs in the given map weight
    weight["F"] = 10 indicates the cost to get to "F" is 10
    record the shortest path to each vertex using given map previous
    previous["F"] = "C" indicates get to "F" via "C"

    cpplint gives warning to use pointer instead of a non-const map
    which I am ignoring for readability */
// This algorithm pushes in a extra first node
// the weights and vertex are correct
//CONFIRMED WITH PROFESSOR - ALLOWED
void Graph::djikstraCostToAllVertices(
    std::string startLabel,
    std::map<std::string, int>& weight,
    std::map<std::string, std::string>& previous) {
    previous.clear();
    weight.clear();
    for (auto &it : vertices) {
        it.second->unvisit();
        weight[it.first] = INT_MAX;
        previous[it.first] = "";
    }
    priority_queue<pair<Vertex*, int>, vector<pair<Vertex*, int>>, 
        greater<>> shortQueue;
    Vertex* startVertex = vertices[startLabel];
    if (startVertex == nullptr) {
        cout << "nodes dont exist";
        return;
    }
    shortQueue.push({ startVertex, 0 });
    weight[startLabel] = 0;
    while (!shortQueue.empty()) {
        Vertex* tempVertex = shortQueue.top().first;
        shortQueue.pop();
        if (!tempVertex->isVisited()) {
            tempVertex->visit();
            tempVertex->resetNeighbor();
            string temp = tempVertex->getNextNeighbor();
            while (temp != tempVertex->getLabel()) {
                Vertex* neighborVertex = vertices[temp];
                int weightVal = weight[tempVertex->getLabel()] +
                    tempVertex->getEdgeWeight(temp);
                if (weightVal < weight[temp]) {
                    weight[temp] = weightVal;
                    previous[temp] = tempVertex->getLabel();
                    shortQueue.push({ neighborVertex, weightVal });
                }
                temp = tempVertex->getNextNeighbor();
            }
        }
    }
}


/** helper for depthFirstTraversal */
void Graph::depthFirstTraversalHelper(Vertex* startVertex,
    void visit(const std::string&)) {
    startVertex->visit();
    startVertex->resetNeighbor();
    string temp = startVertex->getNextNeighbor();
    visit(startVertex->getLabel());
    while (temp != startVertex->getLabel()) {
        Vertex* neighborVertex = vertices[temp];
        if (!neighborVertex->isVisited()
            && neighborVertex != nullptr) {
            depthFirstTraversalHelper(neighborVertex, visit);
        }
        temp = startVertex->getNextNeighbor();
    }
}
//ITERATIVE VERSION ALLOWED
///** helper for breadthFirstTraversal */
//void Graph::breadthFirstTraversalHelper(Vertex* startVertex,
//    void visit(const std::string&)) {}

/** mark all verticies as unvisited */
void Graph::unvisitVertices() {
    for (auto& it : vertices) {
        it.second->unvisit();
    }
}

/** find a vertex, if it does not exist return nullptr */
Vertex* Graph::findVertex(const std::string& vertexLabel) const {
    auto it = vertices.find(vertexLabel);
    return it == vertices.end() ? nullptr : it->second;
}

/** find a vertex, if it does not exist create it and return it */
Vertex* Graph::findOrCreateVertex(const std::string& vertexLabel) { 
    Vertex* vertex = findVertex(vertexLabel);
    if (vertex == nullptr) {
        vertex = new Vertex(vertexLabel);
        vertices[vertexLabel] = vertex;
        numberOfVertices++;
    }
    return vertex;
}
