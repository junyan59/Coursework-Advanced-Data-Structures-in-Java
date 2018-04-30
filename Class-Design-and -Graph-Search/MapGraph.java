
package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/*
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
public class MapGraph {
    // Maintain both nodes and edges as you will need to
    // be able to look up nodes by lat/lon or by roads
    // that contain those nodes.
    private HashMap<GeographicPoint, MapNode> pointNodeMap;
    private HashSet<MapEdge> edges;
    /*
     * Create a new empty MapGraph
     */
    public MapGraph() {
        pointNodeMap = new HashMap<GeographicPoint, MapNode>();
        edges = new HashSet<MapEdge>();
    }

    public static void main(String[] args) {
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/* Use this code in Week 3 End of Week Quiz */
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(1, 1);
        GeographicPoint end = new GeographicPoint(8, -1);


        List<GeographicPoint> route = theMap.dijkstra(start, end);
        System.out.println(route);


        List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
        System.out.println(route2);


    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */


    public int getNumVertices() {
        return pointNodeMap.values().size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return pointNodeMap.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return edges.size();
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location == null) {
            return false;
        }
        MapNode n = pointNodeMap.get(location);
        if (n == null) {
            n = new MapNode(location);
            pointNodeMap.put(location, n);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        MapNode n1 = pointNodeMap.get(from);
        MapNode n2 = pointNodeMap.get(to);

        // check nodes are valid
        if (n1 == null)
            throw new NullPointerException("addEdge: pt1:" + from + "is not in graph");
        if (n2 == null)
            throw new NullPointerException("addEdge: pt2:" + to + "is not in graph");

        MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
        edges.add(edge);
        n1.addEdge(edge);

    }

    /**
     * Get a set of neighbor nodes from a mapNode
     *
     * @param node The node to get the neighbors from
     * @return A set containing the MapNode objects that are the neighbors
     * of node
     */
    private Set<MapNode> getNeighbors(MapNode node) {
        return node.getNeighbors();
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal,
                                     Consumer<GeographicPoint> nodeSearched) {

        // Setup - check validity of inputs
        if (start == null || goal == null)
            throw new NullPointerException("Cannot find route from or to null node");
        MapNode startNode = pointNodeMap.get(start);
        MapNode endNode = pointNodeMap.get(goal);
        if (startNode == null) {
            System.err.println("Start node " + start + " does not exist");
            return null;
        }
        if (endNode == null) {
            System.err.println("End node " + goal + " does not exist");
            return null;
        }

        // setup to begin BFS
        HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
        Queue<MapNode> toExplore = new LinkedList<MapNode>();
        HashSet<MapNode> visited = new HashSet<MapNode>();
        toExplore.add(startNode);
        MapNode next = null;

        while (!toExplore.isEmpty()) {
            next = toExplore.remove();

            // hook for visualization
            nodeSearched.accept(next.getLocation());

            if (next.equals(endNode)) break;
            Set<MapNode> neighbors = getNeighbors(next);
            for (MapNode neighbor : neighbors) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    parentMap.put(neighbor, next);
                    toExplore.add(neighbor);
                }
            }
        }
        if (!next.equals(endNode)) {
            System.out.println("No path found from " + start + " to " + goal);
            return null;
        }
        // Reconstruct the parent path
        List<GeographicPoint> path =
                reconstructPath(parentMap, startNode, endNode);

        return path;

    }

    /**
     * Reconstruct a path from start to goal using the parentMap
     *
     * @param parentMap the HashNode map of children and their parents
     * @param start     The starting location
     * @param goal      The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    private List<GeographicPoint>
    reconstructPath(HashMap<MapNode, MapNode> parentMap,
                    MapNode start, MapNode goal) {
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        MapNode current = goal;

        while (!current.equals(start)) {
            path.addFirst(current.getLocation());
            current = parentMap.get(current);
        }

        // add start
        path.addFirst(start.getLocation());
        return path;
    }


    // run tests
    public static void main(String[] args)
    {
        System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");
    }

}