# Probabilistic Roadmap Generation (PRM)

to run the file: python prmplanner.py
----------------------------------------------------------------------------------------------------------------------------------------
Main parameters used for roadmap generation:
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. N (number of random samples of vertices generated): 5000
2. Equal spacing parameter: 3 units
3. Depth of DFS function: 5
4. max_dist (radius of neighbor search) : 15 units
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Major challenges of this assignment:
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. Generating vertices:

- Random uniform sampling was used to generate N = 5000 samples
- Points in the obstacle space were removed
- Points causing clustering were also removed. an equal spacing parameter (=3 units) was used.
- The remaining points were added to the graph and this was followed by edge generation.
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
2. Generating Edges:

- Incremental collision search was implemented to detect and remove edges which were passing through obstacles.
- An ordered set (python list) was used to consider each vertex only once, for edge generation.
- Vertices within a certain radius of any vertex are considered its neighbors.
- Another function was deployed to prevent triangulation within the vertices. This function uses depth first search to detect if there are multiple paths between two vertices. If such connections are detected, the edge is removed from the graph. 
- On implementation of recursion in the depth first search, the "maximum recursion limit reached" error was triggered. Upon attempting to increase the recursion limit using the sys module, the program would crash. In order to overcome this issue, a finite depth of 5 levels was added to the dfs function instead of automatic recursion. 
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
3. Path finding:

- The start and goal vertices were connected to all vertices of the roadmap lying within a radius of 5 units
- The path was determined using the A-star algorithm for path finding

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Remarks:
The prmplanner.py code was tested for multiple roadmap generations and the default as well as several random queries. It was found to be consistent in finding a path from the start node to the end node.
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------













