# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *

disk_robot = True #(change this to False for the advanced extension) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# Below code written by contributors:
# 1. Shubham Horane
# 2. Prateek Sharma
# 3. Rishabh Bhatia
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap, 
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class  

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius, N

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.


    # the roadmap 
    graph = Roadmap()
    global max_dist

    N = 5000                   ## Number of points considered
    even_spacing_parameter = 3  ## used for removing clustering of the uniformly sampled points
    max_dist = 15               ## Max distance considered for obtaining neareset neighbors
 
    
    '''Sampling of Vertices begins'''
    ## Sampling N points in x and y 
    
    xc = np.random.uniform(low=x_limit[0], high=x_limit[1], size=N)
    yc = np.random.uniform(low=y_limit[0], high=y_limit[1], size=N)


    ### Adding all collision free Vertices to a list
    vertices = []
    for i in range(N):
        if collision([xc[i],yc[i]])==False:
            vertices.append([xc[i],yc[i]])



    ### Removing clustering of points
    for v1 in vertices:
        for v2 in vertices:
            if distance(v1,v2) <= even_spacing_parameter and v1!=v2 and v2 in vertices:
                vertices.remove(v2)

    print('Number of vertices in graph:' ,len(vertices))

    ## Adding above vertices to graph
    for v in vertices:
        graph.addVertex(v)


    '''Sampling of Vertices completed'''
    
    ''' Obtaining and connecting edges'''
    cl_set = OrderedSet()
    count = 0
    for item in graph.getVertices():
        dist1_list = []
        dist1_list = nearest_neighbors(graph, item)
        cl_set.add(item)
        
        for nbr in dist1_list:
            if interpolate(nbr[0], item, 4) == False and nbr[0] not in cl_set:           ## interpolate removes collision-edges | checking with cl_set helps avoid triangulation
                if graph.computeConnectedComponents(item, nbr[0]) == True: continue
                graph.addEdge(item,nbr[0],nbr[1])
                count = count + 1

    print('No of edges: ', count)

    ###
    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
    
def find_path(q_start, q_goal, graph):
    path  = [] 
    parent = (-1) * np.ones(1500, dtype = (tuple,2))  ##defined parent array 
    
    ## Retrieving and storing the 2D coordinates of the start and goal configurations
    qs = (q_start[0], q_start[1])
    qg = (q_goal[0], q_goal[1])
    q_s = graph.addVertex(qs)
    q_g = graph.addVertex(qg)
    

    ### Connecting start and goal vertices to graph
    ## connecting to neighboring vertices within in specific radius 
    neighbor_radius = 5
    for v in graph.vertices:
        if distance(v.q, q_s.q) <= neighbor_radius: graph.addEdge(v,q_s, distance(v.q,q_s.q))
        if distance(v.q, q_g.q) <= neighbor_radius: graph.addEdge(v,q_g, distance(v.q,q_g.q))


     # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Initialize
    open_set.put(qs , Value(f=0+distance(q_s.q,q_g.q),g=0))

    ## Main A* while loop
    while len(open_set)>0:
        next , value = open_set.pop()
        g = value.g
        vert = next

        closed_set.add(next)

        if vert[0] == q_goal[0] and vert[1] == q_goal[1]: break

        else:
            ### Obtaining child vertices iteratively
            for v in graph.vertices:
                if v.q[0] == vert[0] and v.q[1] == vert[1]:
                    for edge in v.edges:
                        new_vert = (graph.vertices[edge.id].q[0], graph.vertices[edge.id].q[1])
                        ### Conditionally adding child vertex to open list
                        if new_vert not in closed_set:
                            new_g = g + edge.dist
                            if new_vert not in open_set or open_set.get(new_vert).g > new_g:
                                
                                new_f = new_g + distance(new_vert, qg)
                                
                                parent[int(edge.id)] = (vert[0],vert[1])
                                
                                open_set.put(new_vert, Value(new_f,new_g))

    
    # # For printing path
    child = qg
    while child[0] != qs[0] and child[1] != qs[1]:
        for v in graph.vertices:
            if v.q[0] == child[0] and v.q[1] == child[1]: 
                child_id = int(v.id)
                par = parent[child_id]
                path.append(par)
                child = par
                break
    path.reverse()
    return path   
# ----------------------------------------
# below are some functions that you may want to populate/modify and use above 
# ----------------------------------------

def nearest_neighbors(graph, q):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """
    global max_dist

    nbr_list = []

    for v1 in graph.getVertices():
        if q.id != v1.id and (distance(q.q, v1.q) <= max_dist):
            d = distance(q.q,v1.q)
            nbr_list.append([v1,d])
            
    return nbr_list


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances 
    """


    return None

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """
    dist = np.sqrt(np.power((q1[0] - q2[0]),2) + np.power((q1[1] - q2[1]),2))
    return dist

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """
    for box in obstacles:
        if q[0] > (box.x_min - robot_radius) and q[0] < (box.x_max + robot_radius) and q[1] > (box.y_min - robot_radius) and q[1] < (box.y_max + robot_radius):
            return True
    return False 
   

def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    
    for i in range(1, int(distance(q1.q,q2.q)/stepsize)+1):
        x1 = q1.q[0]
        x2 = q2.q[0]
        y1 = q1.q[1]
        y2 = q2.q[1]

        if (x2-x1)>0: x_init = x1
        if (x2-x1)<=0: x_init = x2
        if (y2-y1)>0: y_init = y1
        if (y2-y1)<=0: y_init = y2
        x3 = x_init + i*(abs(x1-x2)/stepsize)
        y3 = y_init + i*(abs(y1-y2)/stepsize)
        
        ## Check for collision
        if collision([x3,y3]) == True: return True
    return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
