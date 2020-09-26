# obstacles.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Authors: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

# simple class for AABB obstacles
class BoxObstacle(object):

    def __init__(self, points):
        self.points = points # the 4 vertices of the box
        
        xs = []
        ys = []
        for p in self.points:
            xs.append(p[0])
            ys.append(p[1])
        
        #AABB representation
        self.x_min = min(xs)
        self.x_max = max(xs)
        self.y_min = min(ys) 
        self.y_max = max(ys)

        self.width = self.x_max - self.x_min
        self.height = self.y_max - self.y_min
