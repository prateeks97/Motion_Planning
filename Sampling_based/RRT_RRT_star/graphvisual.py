# graph.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Authors: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

class VisualVertex:
    """A class for storing the vertices of your roadmap """

    def __init__(self, q, id, g, p):
        self.q = q # the configuration of the roadmap vertex
        self.edges = [] # the neighboring roadmap vertices
        self.id = id # the id of the vertex
        self.g = g
        self.p = p
        self.connetedComponentNr = -1 #The id of the component to which the vertex belongs

    def getConfiguration(self):
        return self.q

    def getCost(self):
        return self.g

    def getParent(self):
        return self.p

    def getId(self):
        return self.id

    def getEdges(self): # Returns all edges outgoing from the current vertex
        return self.edges

    def getEdge(self, v_id): # Determines whether an outgoing edge exists between the current vertex and a given vertex
        for e in self.edges:
            if e.getId()==v_id: return e
        return None

    def addEdge(self, v_id, dist, path=[]): # Adds an outgoing roadmap edge from the current vertex to vertex with id v_id
        if self.getEdge(v_id): return False
        self.edges.append(VisualEdge(self.id, v_id, dist, path))
        return True

    def removeEdge(self, v_id): # Removes the edge from the current vertex to a given vertex with id v_id
        for e in self.edges:
            if e.getId()==v_id:
                self.edges.remove(e)
                return True
        return False

    def getConnectedNr(self):
        return self.connetedComponentNr

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.id == other.id


class VisualEdge:
    """ The class defines an edge along the roadmap """

    def __init__(self, src_id, dest_id, dist, path=[]):
        self.id = dest_id # the id of the edge that denotes the target vertex id
        self.dist = dist # the distance associated with the edge
        self.path = path # the local path
        self.src_id = src_id  # the src_id and dest_id are for debugging purposes, all you need is just the id of the edge
        self.dest_id = dest_id

    def getId(self):
        return self.id

    def getDist(self):
        return self.dist

    def getLocalPath(self):
        return self.path

    def getSource(self):
        return self.src_id

    def getDestination(self):
        return self.dest_id

    def setDist(self, dist):
        self.dist = dist
        return self

    def addPathNode(self, q):
        self.path.append(q)
