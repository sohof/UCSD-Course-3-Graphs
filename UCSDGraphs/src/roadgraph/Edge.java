/**
 * @author Sohof Dastmard
 * 
 * A class which represents an edge between two Mapnodes in a graph.
 * Edges are compared to one another based on their lengths. 
 */

package roadgraph;

import geography.GeographicPoint;

public class Edge implements Comparable<Edge> {

	private MapNode from; // from is the tail of the edge
	private MapNode to; // to is the head of the edge thus from --> to
	
	private String roadName;
	private String roadType;
	private double length;
	
	/**
	 * Create a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting MapNode of the edge
	 * @param to The ending MapNode of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 */
	public Edge (MapNode from, MapNode to, String roadName, String roadType, double length){
		
		this.from = from; 
		this.to = to; 
		this.length = length;
		this.roadName = roadName;
		this.roadType = roadType;
		
	}
	
	/**
	 * @return The starting MapNode of the edge 
	 */
	MapNode getFromNode() {
		return from;		
	}
	
	/**
	 * @return The ending MapNode of the edge 
	 */
	MapNode getToNode() {
		return to;		
	}
	
	/**
	 * @param node The node on one side of the edge
	 * @return Given one of the nodes involved in the edge return the other one. Geographic point of the edge 
	 */
	MapNode getOtherNode (MapNode node){
		
		if(node.equals(from))
			return to;
		else if (node.equals(to))
			return to;
		throw new IllegalArgumentException("Looking for a node that is not on the edge");
		
	}
	
	/**
	 * @return The starting Geographic point of the edge 
	 */
	GeographicPoint getFromPoint() {
		return from.getLocation();		
	}
	
	/**
	 * @return The ending Geographic point of the edge 
	 */
	GeographicPoint getToPoint() {
		return to.getLocation();		
	}
	
	
	/**
	 * @return The length of the street (in km) which the edge represents. 
	 */
	public double getLength() {
		return length;
	}
	
	/**
	 * @return The road name the edge represents. 
	 */
	public String getRoadName() {
		return roadName;
	}
	
	/**
	 * @return The road type the edge represents. 
	 */
	public String getRoadType() {
		return roadType;
	}
	
	/**
	 * Fulfilling the Comparable contract in case we need to compare edges.
	 * Compares this edge to that edge based on the length of the two different edges. 
	 * @param that 
	 * @return An integer. Zero if equal. Less than zero if this < that. Greater than zero of this > that
	 */
	public int compareTo(Edge that) {
		
		return Double.compare(this.length,that.getLength());
	}
	
	
	
}
