/**
 * @author UCSD MOOC development team and Sohof Dastmard
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/** 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	
	private HashMap<GeographicPoint, MapNode> pointNodeMap;
	private HashSet<Edge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		pointNodeMap =  new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{	
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{

		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(pointNodeMap.containsKey(location) || location == null)
			return false;
		
		pointNodeMap.put(location, new MapNode(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		MapNode n1 = pointNodeMap.get(from);
		MapNode n2 = pointNodeMap.get(to);

		if (n1 == null)
			throw new NullPointerException("addEdge: the \"from\" node was null") ;
		if (n2 == null)
			throw new NullPointerException("addEdge: the \"to\" node was null");
			

		if(roadType == null || roadName == null || length < 0)
			throw new IllegalArgumentException("addEdge: road:Type/Name/length illegal value");
		
		pointNodeMap.get(from).addEdge(new Edge(n1,n2,roadName,roadType,length));	
		
	}
	
	public void printGraph() {
		
		for(MapNode p : pointNodeMap.values()) {		
			System.out.println(p.getLocation() + "--- >");
			
			for(Edge e : p.getEdges()) {
			
				System.out.println("  " + e.getToPoint() + " length " + e.getLength());
			}
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighed)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 *  The alg. stops at the level the search succeeds and does not cont. expl. the graph.
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighed) path from start to goal
	 *  (including both start and goal). If the search fails after exploring all nodes return null.
	 */
	public List<GeographicPoint> bfs(GeographicPoint s,GeographicPoint g,Consumer<GeographicPoint> nodeSearched)
	{
		// Check input 
		if (s == null || g == null)
			throw new NullPointerException("BFS: from or to Geographic point was null");
		
		MapNode start = pointNodeMap.get(s);
		MapNode goal = pointNodeMap.get(g);
		
		/* Set up to begin bfs	*/
		Queue<MapNode> q = new LinkedList<>();	
		Map<MapNode,MapNode> parentMap = new HashMap<>();	//keep track of parent nodes.
		HashSet<MapNode> visited = new HashSet<>();
		
		boolean found = false;
		visited.add(start);
		q.add(start);
		parentMap.put(start, null); // start node has null as parent
	
		while(!q.isEmpty() && !found) {
			
			MapNode fromNode = q.remove();
			nodeSearched.accept(fromNode.getLocation());
			for(Edge e : fromNode.getEdges() ) {
				
				MapNode toNode = e.getToNode();
				
				
				if (goal.equals(toNode))  //using inherited equals method of class java.awt.geom.Point2D.
					found = true; // if goal equals to node we are done.
					
				if(!visited.contains(toNode)) {	// if we have not already seen this node
					
					parentMap.put(toNode, fromNode);
					visited.add(toNode);
					q.add(toNode);			// add it to the queue to be explored later				
				}
			}		
		}
		
		if (found == true)	
			return constructPath(parentMap, start, goal);
		else
			return null;
	}
	
	private List<GeographicPoint> constructPath(Map<MapNode,MapNode> parentMap,
			MapNode start, MapNode goal) {
		
		// if any of the arguments happens to be null return null to indicate sth is wrong
		if(parentMap == null || start == null || goal == null) {
			return null;
		}
		
		List<GeographicPoint> list = new LinkedList<>();
		
		list.add(goal.getLocation());		
			
		MapNode parent; 
		while((parent = parentMap.get(goal)) != null) // getting the parent of the goal node.
		{
			//System.out.println("inside constructPath while loop");
			list.add(parent.getLocation());
			goal = parent;
		}
		Collections.reverse(list); // Reversing the list so we have from Start --to-- Goal
		return list;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint s,GeographicPoint g,Consumer<GeographicPoint> nodeSearched)
	{
		// Check input 
		if (s == null || g == null)
			throw new NullPointerException("Dijkstra: from or to Geographic point was null");
				
		MapNode start = pointNodeMap.get(s);
		MapNode goal = pointNodeMap.get(g);			
		
		//Mapnode's member field distToSource (dist. to start) used as priority in PriorityQueue
		Comparator<MapNode> comparator = new DistToSourceComparator();
		
		// Set up Dijkstra	
		PriorityQueue<MapNode> pq = new PriorityQueue<>(50, comparator);	
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();	//keep track of parent nodes.
		
		for (MapNode node : pointNodeMap.values())
			node.setDistToSource(Double.POSITIVE_INFINITY);
		
		start.setDistToSource(0.0);
		//int nrNodesExplored =0; // Used in debugging
		pq.add(start);
		parentMap.put(start, null); // start node has null as parent
		while(!pq.isEmpty()) {
			MapNode v = pq.remove();   // v --> w
			
			//nrNodesExplored++; // Used in debugging

			//System.out.println("The min node removed from queue: ----------- " + v.getLocation());
			nodeSearched.accept(v.getLocation());
			
			if (goal.equals(v)) {  //using inherited equals method of class java.awt.geom.Point2D.
				//System.out.println("Nodes explored " + nrNodesExplored); // Used in debugging
				return constructPath(parentMap, start, goal); // are done so construct path
			}
			
			for(Edge e : v.getEdges()) {
				MapNode w = e.getToNode();
						
				if(w.getDistToSource() > v.getDistToSource() + e.getLength() ) {
					
				// We must remove w followed by add to update the priorities in the PriorityQueue
				// however this is probably unnecessarily expensive. And could be implemented faster.
					pq.remove(w); 
					w.setDistToSource(v.getDistToSource() + e.getLength());		
					pq.add(w);
					parentMap.put(w, v);
				}
			}
		}
	
		return null;	
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint s,GeographicPoint g,Consumer<GeographicPoint> nodeSearched)
	{
		// Check input 
		if (s == null || g == null)
			throw new NullPointerException("Dijkstra: from or to Geographic point was null");
				
		MapNode start = pointNodeMap.get(s);
		MapNode goal = pointNodeMap.get(g);			
		
		//Mapnode's member field predicatedDistance is used as priority in PriorityQueue
		Comparator<MapNode> comparator = new aStarComparator();
		
		// Set up astar	
		PriorityQueue<MapNode> pq = new PriorityQueue<>(50, comparator);	
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();	//keep track of parent nodes.
		
		for (MapNode node : pointNodeMap.values()) {
			node.setDistToSource(Double.POSITIVE_INFINITY);
			node.setPredicatedDistance(Double.POSITIVE_INFINITY);
		}
		start.setDistToSource(0.0);
		start.setPredicatedDistance(0.0);
		
		pq.add(start);
		parentMap.put(start, null); // start node has null as parent
		//int nrNodesExplored =0; // Used in debugging
		while(!pq.isEmpty()) {
			
			MapNode v = pq.remove();   // v --> w
			//nrNodesExplored++; // Used in debugging

			//System.out.println("The min node removed: ------- " + v.getLocation() + "	Dist To source " +v.getDistToSource());
			nodeSearched.accept(v.getLocation());	
						
			if (goal.equals(v)) {  //using inherited equals method of class java.awt.geom.Point2D.
				//System.out.println("Nodes explored " + nrNodesExplored);// Used in debugging
				return constructPath(parentMap, start, goal); // are done so construct path
			}
		
			for(Edge e : v.getEdges()) {
				
				MapNode w = e.getToNode();
			
				double heuristicCost = w.getLocation().distance(goal.getLocation());
				w.setDistToGoal(heuristicCost);
						
				if(w.getDistToSource() > v.getDistToSource() + e.getLength() + heuristicCost ) {	
					// We must remove w followed by add to update the priorities in the PriorityQueue
					// however this is probably unnecessarily expensive. And could be implemented faster.
					pq.remove(w); 
					// update distToSource which is used as priority in the Prio.Queue
					w.setDistToSource(v.getDistToSource() + e.getLength()) ;
					w.setPredicatedDistance();
					pq.add(w); 
					parentMap.put(w, v);
				}
			}		
		}
		
		return null;
	}

	public TSPPath tspRoute(GeographicPoint homePoint){
		
		// Check input 
		if (homePoint== null) throw new NullPointerException("TSP: HomeNode was null");
		
		MapNode homeNode = pointNodeMap.get(homePoint);
		MapNode currentNode  = homeNode; 
		ArrayList<MapNode> nodes = new ArrayList<>();
		nodes.addAll(pointNodeMap.values());
		currentNode.setVisited(true);
		TSPPath path  = new TSPPath();
		while(!nodes.isEmpty()) {
			
			Edge next = currentNode.edgeToNearestNotVisitedNeighbor();
			
			if (next != null) { // there is still unvisited neighbors
			next.getToNode().setVisited(true);
			//System.out.println("visited node " + next.getToNode().getLocation());
			
			path.add(next);
			nodes.remove(currentNode);
			currentNode = next.getToNode();
			}
			else {// all vertices are visited connect last visited node to homeNode
				
				Edge edgeToHomeNode = currentNode.edgeTo(homeNode);
				
				if (edgeToHomeNode == null ) {
					System.out.println("Last edge from :" + currentNode.getLocation() + " to HomeNode was null");
					return null;
				}
				
				path.add(edgeToHomeNode);
				nodes.remove(currentNode);
			}
		}
		return path;					
	}
	
	public TSPPath TwoOptimize(TSPPath route) {
		
		int eligibleToBeSwapped = route.getSize() - 1; // nr nodes eligible to be swapped.
		// Could be (I am not sure) that we should change nr of eligible nodes and tweak the allowed i, and k parameters
		// didn't do it because of lack time to debugg and fix potential problems
		
		int iteration =0;
		
		while ( iteration < 2 ) // Iterating more than two times does not seem improve
	    {
			boolean foundBetter = false;
			double bestDist = route.getPathLength();
	 
	        for ( int i = 1; i < eligibleToBeSwapped - 1; i++ ) 
	        {
	            for ( int k = i + 1; k < eligibleToBeSwapped; k++) 
	            {
	            	///System.out.println("Calling TwoSwap with i = " + i + " and k = " + k);
	                TSPPath newRoute = TwoOptSwap(route, i, k);
	                
	                double new_distance = newRoute.getPathLength();
	 
	                if ( new_distance < bestDist ) 
	                {
	                      route = newRoute;  
	                      System.out.println("Found better Distance "+ newRoute.getPathLength());
	                      foundBetter = true;
	                      break;
	                }
	            }
	            
	            if (foundBetter)
	            	break;
	        }
	        iteration++;
	    }
		return route;
	}
		
	public TSPPath TwoOptSwap(TSPPath route, int i, int k ) {
		
		int routeSize = route.getSize();
				
		TSPPath newRoute = new TSPPath();
		ArrayList<MapNode> nodes = route.getPathNodes();		
			
		//take route[0] to route[i-1] and add them in order to new_route
		for (int index = 0;  index < i-1 ; ++index ) {

			Edge e = route.getPathEdges().get(index);
			newRoute.add(e);	
		}
		
		// node (i-1) must be connected to node k
		Edge e = nodes.get(i-1).edgeTo(nodes.get(k));
		newRoute.add(e);	

		// take route[i] to route[k] and add them in reverse order to new_route
		for (int index = k;  index > i; index-- ) {

			Edge e2 = nodes.get(index).edgeTo(nodes.get(index-1)); //create reverse edge
			newRoute.add(e2);
		}
		
		// node (i) must be connected to node k+1
		newRoute.add (nodes.get(i).edgeTo(nodes.get(k+1)));
		
		//take route[k+1] to end and add them in order to new_route
		for (int index = k+1;  index < routeSize; ++index ) {
			newRoute.add(route.getPathEdges().get(index));
		}		
	
		return newRoute;
	}
	
	public static void main(String[] args)
	{
		/* TESTING Graph construction and the BFS algorithm */
		
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//		
//		System.out.println("Printing the constructed graph.");
//		firstMap.printGraph();
//		System.out.println("Printing graph DONE.");
//		
//		System.out.println("Testing bfs search on nodes (1.0,1.0) to (8.0, -1.0) .");
//
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		List<GeographicPoint> l = firstMap.bfs(testStart, testEnd);
//		
//		if(l != null) {
//			for (GeographicPoint p : l) 
//			{
//				System.out.println(p);
//			}
//		}
//		else if (l == null)
//			System.out.println("null");
		
		
		/* Test cases for dijkstra and astar search	*/
		
	/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);	
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		//simpleTestMap.printGraph();  // For debugging the grapg
		System.out.println("Test 1 using simpletest.Nodes Explored: Dijkstra should be 9 and AStar should be 5");

		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println("Dijkstra nr of nodes in shortest path = " + testroute.size()); 
		
		for (GeographicPoint p : testroute) {
				System.out.println(p);
				
		}
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);

		System.out.println("aStar nr of nodes in shortest path = " + testroute2.size());
		for (GeographicPoint p : testroute2) {
			System.out.println(p);		
		}
	*/
		
		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc. Nodes Explored: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		System.out.println("Nr of nodes in shortest path. Dijkstra = " + testroute.size() + ", aStar = " + testroute2.size());

		
		
		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc.Nodes Explored: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		System.out.println("Nr of nodes in shortest path. Dijkstra = " + testroute.size() + ", aStar = " + testroute2.size());

		
		
		/* Use this code in Week 3 End of Week Quiz */
		
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//		
//		
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		/* Testing TSP gready algorithm, own extension for week 6 */
		// The map used is the same as simpletest.map, only modified to be a complete graph.
		
		System.out.print("Making a new map for testing TSP...");
		MapGraph tspMap = new MapGraph();
		System.out.print("DONE. \nLoading complete graphsimpleTSPtest...");
		GraphLoader.loadRoadMap("data/testdata/simpleTSPtest.map", tspMap);
		System.out.println("DONE.");

		System.out.println("Testing TSP with HomeNode 1.0,1.0.");
		System.out.println("The Nearest Neighbor Algorithm Found:");

		// We will get different solutions based on using different homeNodes!
		// We could do a for loop and look for solutions using all the different nodes as homeNode
		GeographicPoint homeNode = new GeographicPoint(1.0, 1.0);
		TSPPath route = tspMap.tspRoute(homeNode);
		
		if(route != null)
			route.printTSPPath();
		else 
			System.out.println("Error route was null");
		
		System.out.println("Applying 2-Opt Optimization to Nearest Neighbor Solution Found:");
		route = tspMap.TwoOptimize(route);
		System.out.println("Route was optimized. Current best route:");
		if(route != null)
			route.printTSPPath();
		
		
	}// end main
	
}
