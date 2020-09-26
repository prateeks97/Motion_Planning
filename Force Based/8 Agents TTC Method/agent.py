# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=4, goalRadiusSq=1, maxF = 10):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = 0.5 # the relaxation time used to compute the goal force
        self.dhor = 10 # the sensing radius
        self.timehor = 4 # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = 10 # the maximum force that can be applied to the agent



    def computeForces(self, neighbors=[]):
        E = 0.2


        if not self.atGoal:
            F_total = np.zeros(2)
            self.F = (self.gvel-self.vel)/self.ksi #Driving force calculation
            new_neighbors=[]
            """
            Determining all neighbors within sensing area
            """
            for n in neighbors:
                dist= np.sqrt(((n.pos[0] - self.pos[0])**2)+((n.pos[1] - self.pos[1])**2))
                if dist == 0:
                    continue
                elif (dist <= self.dhor):
                    if (np.sqrt(((self.goal[0]-self.pos[0])**2)+((self.goal[1]-self.pos[1])**2))<self.goalRadiusSq):    # to stop appending neighbors when self.pos is within 1meter from goal
                        continue
                    new_neighbors.append([n.pos[0], n.pos[1], n.vel[0], n.vel[1], n.radius, n.id])


            """
            Calculate time to collision
            """
            if (dist<=self.dhor):
                for num in new_neighbors:
                    xa=np.array([self.pos[0], self.pos[1]])
                    xb=np.array([num[0], num[1]])
                    x=xa-xb
                    va=np.array([self.vel[0], self.vel[1]])
                    vb=np.array([num[2], num[3]])
                    v=(va-vb)
                    rad=self.radius + num[4]
                    a=np.dot(v,v) - E**2
                    b=np.dot(x,v) - E*rad
                    c=np.dot(x,x)-(rad*rad)
                    d=b*b-a*c

                    """
                    Computing various conditions to calculate admissible time to collision
                    """
                    if c<0:
                        tau=0
                    if b>0:
                        continue
                    if d<=0:
                        continue
                    tau = c/(-b + np.sqrt(d))
                    if tau<=0:
                        continue
                    """
                    Calculating avoidance force
                    """
                    if tau>0:
                        n_ab = np.array([(x[0]+v[0]*tau)/(sqrt((x[0]+v[0]*tau)**2+(x[1]+v[1]*tau)**2)),(x[1]+v[1]*tau)/(sqrt((x[0]+v[0]*tau)**2+(x[1]+v[1]*tau)**2))])
                        F_avoid = ((max(self.timehor-tau,0))/tau)*n_ab
                        F_total += F_avoid

                self.F += F_total



    def update(self, dt):
        """
            Code to update the velocity and position of the agents.
            as well as determine the new goal velocity
        """
        if not self.atGoal:

            if (sqrt(self.F[0]**2+self.F[1]**2)>self.maxF):
                self.F=(self.F/(sqrt(self.F[0]**2+self.F[1]**2)))*self.maxF
            self.vel += self.F*dt     # update the velocity
            if (sqrt(self.vel[0]**2+self.vel[1]**2)>self.maxspeed):
                self.vel=(self.vel/(sqrt(self.vel[0]**2+self.vel[1]**2)))*self.maxspeed
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
