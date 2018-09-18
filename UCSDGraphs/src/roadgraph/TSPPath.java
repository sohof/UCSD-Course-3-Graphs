/**
 * @author Sohof Dastmard
 * 
 * A class which represents an Traveling salesman path.
 * Edges are compared to one another based on their lengths. 
 */
package roadgraph;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


public class TSPPath {
	
	private double pathLength; // the total distance of all edges involved 
	private int pathSize;	// nr of nodes/edges on this path

	ArrayList<Edge> pathEdges;
	ArrayList<MapNode> pathNodes;
	
	public TSPPath() {
		
		pathEdges = new ArrayList<>();
		pathNodes = new ArrayList<>();
		pathLength = 0;
		pathSize = 0;
	}
	
	public void printTSPPath() {
		
		System.out.print("Path total length: " + pathLength);
		System.out.println(". Nr PathNodes: " + pathNodes.size() + ". Nr PathEdges: " +pathEdges.size());
		
		for(int i = 0;  i < pathSize; i++) {
			
			Edge e = pathEdges.get(i);
			MapNode node = pathNodes.get(i);
			System.out.println("Edge: " + e.getFromPoint() + " --> " + e.getToPoint());
		}
					
	}
	
	public int getSize() { return pathSize;}
	
	public void add(Edge e) {

		if(e == null) throw new IllegalArgumentException("Trying to add a null edge");
		
		pathEdges.add(e);
		 // we add the mapnode from where we came. This means when adding the last edge that
		// connects back to the homeNode. The homenode will not be added again to the path,
		// it is just added once, when the first edge is added to the path.
		pathNodes.add(e.getFromNode());
		pathLength += e.getLength();
		pathSize++;
	}

	double getPathLength() {
		return pathLength; 
	}

	ArrayList<Edge> getPathEdges() {
		return pathEdges;
	}
	
	ArrayList<MapNode> getPathNodes() {
		return pathNodes;
	}
	
}
