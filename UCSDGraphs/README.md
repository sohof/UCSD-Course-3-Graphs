## Starter Code and GUI Application for Course 3 in the
## Java Programming: Object Oriented Design of 
## Data Structures Specialization:
## Advanced Data Structures in Java
## https://www.coursera.org/learn/advanced-data-structures


# UCSD-Course-3-Graphs. Course 3 UC San Diego, Advanced Data Structures.

The project for this course is to build a mapping application with the ability to provide and visualize routes from one point to another in a map. The code developed is “back-end” code, i.e the data structures and algorithms used. A front-end interface was provided by USCD. The GUI is written using the Java FX libraries

Designed classes to implementd Graphs and basic graph search algorithms including BFS, DFS, Dijkstra's Algorithm, and A* Search. Also a travelling sales man implementation with the ability to provide and visualize routes from one point to another in a map. The tsp algorithm used was nearestNeighbor followed by 2-Optimize technique.

There is also a folder named Documentation a few selected pdfs are stored for more information about the course.
The schedule was the following:

## Week 1 and 2
Worked on the following classes: an abstract class Graph.java, and then two concrete implementations extensions of that graph class. GraphAdjMatrix.java and GraphAdjlist.java.

## Week 3
Worked on/Designed: MapGraph.java MapNode.java Edge.java and added BFS search to method to the MapGraph class.

## Week 4
Implemented dijkstra and A* search methods in the MapGraph class

## Week 5 and 6
Theory on Route planning and NP-hard graph problems. Worked on an extention to the project. Choose to work on finding Travelling Salesman Path in the graph. Added the support class TSPPath.java and tspRoute method in the MapGraph class. The tsp algorithm uses was nearestNeighbor followed by 2-Optimize.


# Overview starter code and packages

Most of the starter code is dedicated to implementing the front-end, and you will not have to use it at all. The following are the packages was worked with in this course: (only listing classes in the packages that I worked with)
## basicgraph:
Abstract class Graph.java and two concrete implementations extensions of that graph class. GraphAdjMatrix.java and GraphAdjlist.java. s

## geography:

Contains two classes, but you will only work with the class GeographicPoint. You should not modify this class, but you will need to use it in your assignments in weeks 3 and 4 and 6.

## roadgraph:
This package was worked with in weeks 3, 4 and 6. Moodified provided code as well as added classes to this package. MapGraph.java MapNode.java Edge.java TSPPath.java

## util:
Contains a utility class, GraphLoader, for loading data from files. You will need to use this class for testing, but you should not modify it.

## week3example:
Contains the example code from week 3. Feel free to run this code as well as modify it. You should not need to touch any of the other packages, but if you are curious, you are more than welcome to poke around.


# Overview of the data files

In addition to the starter code, there is also some data to begin working with, as well as a mechanism for collecting your own road data (explained in the last section of this reading).

All provided data can be found in the data directory (at the same level as the src directory). In this directory you will find several sub-folders. Here's an explanation of what is in each:

## airports:
Data about airports and routes served by United Airlines. These files use the extension .dat. You can load this data file into an object of type basicgraph.Graph using the GraphLoader.loadRoutes method.
graders:

Copies of the graph files used for grading each week. You can look at these files, but do not change these files or you will not be able to run our graders on your code to see what might have gone wrong.

## intersections:
Data suitable for upload into the map visualization tool in week 3. These files use the extension .intersections. You can generate files of this type from raw map files (see below) using the method GraphLoader.createIntersectionsFile. Be sure to save your newly created files into this same intersections folder and give them the .intersections extension so you keep track of their format. More on the process of generating these files can be found in the next section.


## maps:
Raw map data files that are saved from the front-end data fetcher. These files use the extension .map. You can load these files into an empty roadgraph.MapGraph object or into an empty basicgraph.Graph object using the GraphLoader.loadRoadMap method (it's overloaded for both types). We provide several raw maps from various locations in the USA, but we encourage you to collect your own data using our application. More on this process can be found in the next section.

## mazes:
Maze files used in the example in week 3. These files use the extension .maze. We encourage you to create your own. You can load these files using the MazeLoader utility in the week2example package. testdata: An artificial map for use in testing in weeks 2, 3, 4 and 6. It is in the same .map format as the files in the maps folder. You can load this file into an empty roadgraph.MapGraph object or into an empty basicgraph.Graph object using the GraphLoader.loadRoadMap method (it's overloaded for both types).



## ----------[ DESCRIPTION README BY COURSERA ]---------------

The files provided are skeleton code, as well as grading previews and 
testing files to be used in completing the course programming 
assignments. Additionally, you are provided a runnable JavaFX program 
which will help to test and demonstrate your implementations.

## --[ FILES BY WEEK ]--

Below are the files introduced in each week and used in each week
of the course. See file for description...

Week 1 : Introduction to the course and graphs
==============================================
basicgraph.Graph.java
basicgraph.GraphAdjList.java
basicgraph.GraphAdjMatrix.java

Week 2 : Class design and simple graph search
==================================================
roadgraph.MapGraph.java
week2example.Maze.java
week2example.MazeLoader.java
week2example.MazeNode.java

Utility files
=============
geography.GeographicPoint.java
geography.RoadSegment.java
util.GraphLoader.java

## --[ SETUP ]-- 

Importing Project into eclipse:
	1. Create a new Java Project in your workspace
	2. Import the starter files:
	  File -> Import -> Select "File System" -> Next -> Browse and set 
	  root directory to folder contents of zip were extracted to -> Finish

Feel free to use another IDE or manually compile and run your programs.
If you need help, google is your friend.
