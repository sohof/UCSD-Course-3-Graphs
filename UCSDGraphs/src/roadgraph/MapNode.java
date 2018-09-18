/**
 * @author Sohof Dastmard
 * 
 * A class which represents a node in the graph. A node is geographic point in a map.
 * Each node has a hashset of edges. The nodes are used in several different algorithms
 * and thus have extra support structure.  
 */

package roadgraph;

import java.util.Set;
import java.util.Comparator;
import java.util.HashSet;
import geography.GeographicPoint;

public class MapNode {
	
	/* A boolean to mark if this node has been visited or not. For use in the TSP alg.*/
	private boolean isVisited = false;
	
	/* The list of edges out of this node*/
	private HashSet<Edge> edges;
	
	/* Distance between this node to a given goal/target node. Used in  dijkstra and a-star*/
	private double distToSource = 0; 
	
	/* Straight line distance from this node to a goal node. It is the heuristic used in astar algorithm */
	private double distToGoal = 0; 
	
	/* The predicated distance is the f-function in the astar algorithm. Which is used to guide our priority queue
	 * For each node we let distToSource + distToGoal heuristic guide which node to explore first from our queue */
	private double predicatedDistance = 0; 
	
	/* The latitude and longitude of this node*/
	private GeographicPoint location;
	
	/** 
	 * Create a new MapNode at a given Geographic location
	 * @param loc the location of this node
	 */
	MapNode(GeographicPoint loc){
		
		location = loc;
		edges = new HashSet<Edge>();
	}

	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	void addEdge(Edge edge) { edges.add(edge); }
	
	/**
	 * Get the geographic location that this node represents
	 * @return the geographic location of this node
	 */
	GeographicPoint getLocation() {return location;}
	
	/**
	 * return the edges out of this node
	 * @return a set containing all the edges out of this node.
	 */
	Set<Edge> getEdges() {return edges;}
	
	/**
	 * The method checks if there is an edge from this MapNode to the supplied MapNode 
	 * @param the MapNode to find an edge to 
	 * @return the edge, to given MapNode, or null if no such edge.
	 */
	public Edge edgeTo(MapNode argNode) {
		
		Edge edge = null;
		for (Edge e : edges)
		{
			if(e.getToNode().equals(argNode))
				edge = e;
		}
		return edge;
	}
	
	/**
	 * A reference to this nodes nearest not visited neighbor. For use in the tspRoute method.
	 * @return a MapNode, the nearest (in terms of edge length) not visited neighbor of this node.
	 */
	public Edge edgeToNearestNotVisitedNeighbor() {
		
		double minEdgeLength = Double.POSITIVE_INFINITY;
		Edge minEdge = null;
		
		for (Edge e : edges)
		{
			if (e.getLength() < minEdgeLength && e.getToNode().isVisited() == false )
			{
				minEdge = e;
				minEdgeLength = e.getLength();
			}
			
		}
		
		if (minEdge != null)
			return minEdge;
		else
			return null;
	}
	
	/** Returns whether two nodes are equal. Nodes are considered equal if their locations are the same, 
	 * even if their street list is different. The location object uses the inherited equals method 
	 * of class java.awt.geom.Point2D. Which compares the X and Y coordinates.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** 
	 * @return The distance to a designated source/start node. 
	 */
	double getDistToSource() { return distToSource; }
	
	/** 
	 * @param The distance to the source node.  
	 */
	void setDistToSource(double distance) { distToSource = distance; }
	
	/** 
	 * @return The distance to a designated goal node. 
	 */
	double getDistToGoal() { return distToGoal; }
	
	/** The heuristic used in astar search. The straigt line distance between the nodes. The heuristic
	 * must always underestimate the distance/cost, for astar to work properly.
	 * @param The distance to the source node.  
	 */
	void setDistToGoal(double distance) { distToGoal = distance; }
	
	/** 
	 * @return The predicated distance to a designated goal node. 
	 */
	double getPredicatedDistance() {return predicatedDistance ;}  
	
	void setPredicatedDistance() {predicatedDistance = distToSource + distToGoal;} 
	
	/** This overloaded is used if we need to set it manually. Usually in a initialization phase.
	 * @param The predicated distance to a designated goal node. 
	 */	
	void setPredicatedDistance(double initialValue) {predicatedDistance = initialValue ;} 
	
	
	boolean isVisited() {return isVisited;}
	void setVisited(boolean visited) {isVisited = visited;}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (Edge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}
	
}

class DistToSourceComparator implements Comparator<MapNode>{

	@Override
	public int compare(MapNode node1, MapNode node2) {
		
		if (node1.getDistToSource() < node2.getDistToSource())
			return -1;
		if (node1.getDistToSource() > node2.getDistToSource())
			return 1;
				
		return 0;
	}
}
	
class aStarComparator implements Comparator<MapNode>{

		@Override
		public int compare(MapNode node1, MapNode node2) {

			if (node1.getPredicatedDistance() < node2.getPredicatedDistance())
				return -1;
			if (node1.getPredicatedDistance() > node2.getPredicatedDistance())
				return 1;
					
			return 0;
		}	
	
}

