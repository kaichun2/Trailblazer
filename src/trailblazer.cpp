// This program displays various 2-dimensional worlds that represent either maps, mazes, or terrain and allows the user to generate paths in a world from one point to another.

#include "trailblazer.h"
#include "queue.h"
#include "hashset.h"
#include "pqueue.h"
#include "stack.h"

using namespace std;

bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path);
void breadthFirstSearchHelper(BasicGraph& graph, Vertex* end, Queue<Vector<Vertex*>>& pathQueue, Vector<Vertex*>& path);
bool dijkstrasAlgorithmHelper(BasicGraph& graph, Vertex* end, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, PriorityQueue<Vertex*>& vertexPQ);
void addOrReplaceCost(double newCost, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, Vertex* currentVertex, Vertex* start, PriorityQueue<Vertex*>& vertexPQ, double aStarValue);
void updateCost(HashMap<Vector<Vertex*>, double>& vertexCurrentCost, Vertex* currentVertex, double newCost, PriorityQueue<Vertex*>& vertexPQ, Vertex* start, double aStarValue);
bool aStarHelper(BasicGraph& graph, Vertex*& end, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, PriorityQueue<Vertex*>& vertexPQ);
void kruskalHelper(PriorityQueue<Edge*>& edgePQ, Set<Edge*>& mst, HashSet<HashSet<Vertex*>>& clusterCollection);
void updateCluster(Set<Edge*>& mst, Edge* currentEdge, Vertex* vertexToCheck, HashSet<Vertex*>& oldCluster, HashSet<Vertex*>& newCluster, HashSet<HashSet<Vertex*>>& clusterCollection);
void checkIfNeedToCombineCluster(HashSet<HashSet<Vertex*>>& clusterCollection, Vertex* vertexToCheck, HashSet<Vertex*>& newCluster);

// Finds a path between two vertices by exploring each possible path. Set the impossible path into grey.
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path = {start};
    start->setColor(GREEN);
    if (! depthFirstSearchHelper(graph, start, end, path)){
        start->setColor(GRAY);
        // return empty path if no path found.
        path.remove(0);
    }
    return path;
}

// Return true if a path is found. Explore each possible path as far as possible before backtracking.
bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*>& path) {
    if (start == end) {
        // Return true instantly if a path is found
        return true;
    }
    for (Vertex* vertex: graph.getNeighbors(start)){
        // Check whether the neighbour has been visited.
        if (vertex->getColor() != GRAY && vertex->getColor() != GREEN) {
            // Mark the neighbour as visited and add to the path
            vertex->setColor(GREEN);
            path.add(vertex);
            // Go to next neighbor if it cannot find a path.
            if (! depthFirstSearchHelper(graph, vertex, end, path)) {
                vertex->setColor(GRAY);
                path.remove(path.size() - 1);
            } else {
                // Return true instantly if a path is found
                return true;
            }
        }
    }
    // Return false if it cannot find a path.
    return false;
}

// Finds a path between two nodes by taking one step down all paths.
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    // Vertex for storing the final path.
    Vector<Vertex*> path = {start};
    start->setColor(GREEN);
    if (start != end) {
        // A queue storing all the paths that ends in different vertex.
        Queue<Vector<Vertex*>> pathQueue;
        pathQueue.enqueue(path);
        breadthFirstSearchHelper(graph, end, pathQueue, path);
        if (pathQueue.size() == 0) {
            // return empty path if no path found.
            path.remove(0);
        }
    }
    return path;
}

// Finds a path between two nodes by taking one step down all paths and then immediately backtracking.
void breadthFirstSearchHelper(BasicGraph& graph, Vertex* end, Queue<Vector<Vertex*>>& pathQueue, Vector<Vertex*>& path){
    // pathQueue.size() is zero if it cannot find a path.
    if (pathQueue.size() != 0) {
        Vector<Vertex*> currentPath = pathQueue.dequeue();
        // Get the previous end vertex from the currentPath.
        Vertex* start = currentPath[currentPath.size() - 1];
        // Set it as visited.
        start->setColor(GREEN);
        if (start != end) {
            // Explore the neighbor.
            for (Vertex* vertex : graph.getNeighbors(start)){
                //check whether the neighbour has been visited
                if (vertex->getColor() != YELLOW && vertex->getColor() != GREEN) {
                    vertex->setColor(YELLOW);
                    currentPath.add(vertex);
                    pathQueue.enqueue(currentPath);
                    // Remove the last element of the current path.
                    currentPath.remove(currentPath.size() - 1);
                }
            }
            // Explore the neighbors from the current vertex.
            breadthFirstSearchHelper(graph, end, pathQueue, path);
        } else {
            // Stop the recursion when a path is found.
            path = currentPath;
        }
    }
}

// Finds the minimum-weight path between a pair of vertices in a weighted directed graph.
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path = {start};
    PriorityQueue<Vertex*> vertexPQ;
    // 0 cost from the start vertex.
    vertexPQ.enqueue(start, 0);
    start->setColor(GREEN);
    if (start != end) {
        // Map with key being vertex path (first one is start, last one is end), with value being accumuated cost.
        // For memorizing the path and its current cost.
        HashMap<Vector<Vertex*>, double> vertexCurrentCost;
        // First put starting vertex with zero cost.
        vertexCurrentCost.put({start}, 0);
        if (dijkstrasAlgorithmHelper(graph, end, vertexCurrentCost, vertexPQ)) {
            // Go to this loop if it found a path.
            // Loop through the map to return the path we want
            for (Vector<Vertex*> path: vertexCurrentCost) {
                if (path[path.size() - 1] == end) {
                    return path;
                }
            }
        } else {
            // Return empty path if it cannot find a path.
            path.remove(0);
        }
    }
    return path;
}

// Return true if it can find a path.
bool dijkstrasAlgorithmHelper(BasicGraph& graph, Vertex* end, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, PriorityQueue<Vertex*>& vertexPQ) {
    if (! vertexPQ.isEmpty()) {
        double currentCost = vertexPQ.peekPriority();
        Vertex* start = vertexPQ.dequeue();
        start->setColor(GREEN);
        if (start != end) {
            for (Edge* currentEdge : graph.getEdgeSet(start)){
                Vertex* currentVertex = currentEdge->finish;
                //check whether the neighbour has been visited
                if (currentVertex->getColor() != GREEN) {
                    if (currentVertex->getColor() != YELLOW) {
                        currentVertex->setColor(YELLOW);
                    }
                    // get the cost from start to the neighbor vertex, and add it to the existing cost.
                    double newCost = currentCost + currentEdge->cost;
                    // Check whether a path to current vertex exists already.
                    // the last parameter is non zero only when we are using A*.(The zero wont be used in this algorithm.
                    addOrReplaceCost(newCost, vertexCurrentCost, currentVertex, start, vertexPQ, 0);
                }
            }
            return dijkstrasAlgorithmHelper(graph, end, vertexCurrentCost, vertexPQ);
        } else {
            // return true when a path is found.
            return true;
        }
    }
    // return false when PQ is empty, which means no path is found.
    return false;
}

// Check whether a path to current vertex exists already.
void addOrReplaceCost(double newCost, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, Vertex* currentVertex, Vertex* start, PriorityQueue<Vertex*>& vertexPQ, double aStarValue) {
    bool existInPQ = false;
    for (Vector<Vertex*> path: vertexCurrentCost) {
        // if a path to current vertex exists already and old cost > new cost, update the cost.
        if (path[path.size() - 1] == currentVertex) {
            if (vertexCurrentCost[path] > newCost) {
                vertexCurrentCost.remove(path);
                updateCost(vertexCurrentCost, currentVertex, newCost, vertexPQ, start, aStarValue);
            }
            existInPQ = true;
            break;
        }
    }
    // If the current vertex doesnt exist in the pq, add it.
    if (! existInPQ) {
        vertexPQ.add(currentVertex, newCost + aStarValue);
        updateCost(vertexCurrentCost, currentVertex, newCost, vertexPQ, start, aStarValue);
    }
}

// For updating the cost in the map. A star value is the heuristicFunction double that needed to be added into the PQ.
// astar value  will be 0 if dijkstrasAlgorithm is used.
void updateCost(HashMap<Vector<Vertex*>, double>& vertexCurrentCost, Vertex* currentVertex, double newCost, PriorityQueue<Vertex*>& vertexPQ, Vertex* start, double aStarValue) {
    for (Vector<Vertex*> path: vertexCurrentCost) {
        // Get the path that ends with the previous starting vertex and add the current vertex into it.
        if (path[path.size() - 1] == start) {
            Vector<Vertex*> newPath = path;
            newPath.add(currentVertex);
            vertexCurrentCost.put(newPath, newCost);
            vertexPQ.changePriority(currentVertex, newCost + aStarValue);
            break;
        }
    }
}

// A modified version of Dijkstra's algorithm that uses a heuristic function to guide its order of path exploration.
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path = {start};
    PriorityQueue<Vertex*> vertexPQ;
    vertexPQ.enqueue(start, heuristicFunction(start, end));
    start->setColor(GREEN);
    if (start != end) {
        // Map with key being vertex path (first one is start, last one is end), with value being accumuated cost.
        HashMap<Vector<Vertex*>, double> vertexCurrentCost;
        vertexCurrentCost.put({start}, 0);
        if (aStarHelper(graph, end, vertexCurrentCost, vertexPQ)) {
            for (Vector<Vertex*> path: vertexCurrentCost) {
                if (path[path.size() - 1] == end) {
                    return path;
                }
            }
        } else {
            // Remove start vertex from path if it cannot find a path.
            path.remove(0);
        }
    }
    return path;
}

// Similar to dijkstrasAlgorithmHelper, but uses a heuristic function to guide its order of path exploration.
bool aStarHelper(BasicGraph& graph, Vertex*& end, HashMap<Vector<Vertex*>, double>& vertexCurrentCost, PriorityQueue<Vertex*>& vertexPQ){
    if (! vertexPQ.isEmpty()) {
        double currentExpectedCost = vertexPQ.peekPriority();
        Vertex* start = vertexPQ.dequeue();
        start->setColor(GREEN);
        if (start != end) {
            for (Edge* currentEdge : graph.getEdgeSet(start)){
                Vertex* currentVertex = currentEdge->finish;
                //check whether the neighbour has been visited
                if (currentVertex->getColor() != GREEN) {
                    if (currentVertex->getColor() != YELLOW) {
                        currentVertex->setColor(YELLOW);
                    }
                    // New cost for adding into the map.
                    double newCost = currentExpectedCost - heuristicFunction(start, end) + currentEdge->cost;
                    // This aStarValue will be added with newCost to the PQ.
                    double aStarValue = heuristicFunction(currentVertex, end);
                    addOrReplaceCost(newCost, vertexCurrentCost, currentVertex, start, vertexPQ, aStarValue);
                }
            }
            return aStarHelper(graph, end, vertexCurrentCost, vertexPQ);
        } else {
            // return true when a path is found.
            return true;
        }
    }
    return false;
}

Set<Edge*> kruskal(BasicGraph& graph) {
    Set<Edge*> mst;
    // HashSet collecting different clusters
    HashSet<HashSet<Vertex*>> clusterCollection;
    PriorityQueue<Edge*> edgePQ;
    //construct a priority queue of all edges based on their weight.
    for (Edge* edge: graph.getEdgeSet()) {
        edgePQ.enqueue(edge, edge->cost);
    }
    kruskalHelper(edgePQ, mst, clusterCollection);
    return mst;
}

void kruskalHelper(PriorityQueue<Edge*>& edgePQ, Set<Edge*>& mst, HashSet<HashSet<Vertex*>>& clusterCollection) {
    while (! edgePQ.isEmpty()) {
        Edge* currentEdge = edgePQ.dequeue();
        bool bothVertexExist = false;
        HashSet<Vertex*> newCluster;
        Vertex* vertexToCheck = nullptr;
        // Loop through all the clusters to find whether start or end vertex of the dequeued edge exists in any cluster.
        for (HashSet<Vertex*> cluster: clusterCollection) {
            // If cluster have both start and end, do nothing.
            if (cluster.contains(currentEdge->start) && cluster.contains(currentEdge->end)) {
                bothVertexExist = true;
                break;
            }
            // If the cluster contains start or end vertex (either one only), check if needed to combine clusters.
            if (cluster.contains(currentEdge->start) || cluster.contains(currentEdge->end)) {
                // If cluster contains start vertex only, remember the vertexToCheck is currentEdge->end.
                if (cluster.contains(currentEdge->start)) {
                    vertexToCheck = currentEdge->end;
                } else {
                    // If cluster contains end vertex only, remember the vertexToCheck is currentEdge->start.
                    vertexToCheck = currentEdge->start;
                }
                updateCluster(mst, currentEdge, vertexToCheck, cluster, newCluster, clusterCollection);
                break;
            }
        }
        // Don't do anything if a cluster contains both start and end vertex.
        if (! bothVertexExist) {
            // If no clusters contain start or end vertex (then vertexToCheck will be null), create a new cluster.
            if (! vertexToCheck) {
                mst.add(currentEdge);
                clusterCollection.add({currentEdge->start, currentEdge->end});
            } else {
                // Else, check if another vertex exists in another cluster.
                checkIfNeedToCombineCluster(clusterCollection, vertexToCheck, newCluster);
            }
        }
    }
}

// Update the cluster by adding the new vertex into a new cluster and replace the old cluster with the new one.
// For the replace action -> The remove action is done here and the add action will be done in checkIfNeedToCombineCluster function.
void updateCluster(Set<Edge*>& mst, Edge* currentEdge, Vertex* vertexToCheck, HashSet<Vertex*>& oldCluster, HashSet<Vertex*>& newCluster, HashSet<HashSet<Vertex*>>& clusterCollection) {
    mst.add(currentEdge);
    newCluster += oldCluster;
    newCluster.add(vertexToCheck);
    clusterCollection.remove(oldCluster);
}

// Loop through all clusters to see if we need to conbine any of those.
void checkIfNeedToCombineCluster(HashSet<HashSet<Vertex*>>& clusterCollection, Vertex* vertexToCheck, HashSet<Vertex*>& newCluster) {
    for (HashSet<Vertex*> cluster: clusterCollection) {
        if (cluster.contains(vertexToCheck)) {
            newCluster += cluster;
            clusterCollection.remove(cluster);
            clusterCollection.add(newCluster);
            return;
        }
    }
    // Create a new and replace the old cluster with the new one. (Old cluster had been removed already in updateCluster funcion.)
    clusterCollection.add(newCluster);
}
