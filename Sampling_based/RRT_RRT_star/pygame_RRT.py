#!/usr/bin/env python3
import reeds_shepp_path
from reeds_shepp_path import ReedsSheppPath
from reeds_shepp_path import draw_point
from math import pi


# Import the pygame module
import numpy as np
from scene import *
import time
import functools
from graph import*
import random
from math import sqrt
import pygame
import pygame.gfxdraw
from dubins import Dubins, dist
import matplotlib
matplotlib.use("Agg")
import matplotlib.backends.backend_agg as agg
import pylab
from pygame.locals import *
import os

# choice = input("what do you want to run?")
algo_choice = int( input("1. RRT \n2. RRT* \nEnter the number: "))
if algo_choice ==1:
    choice = "rrt"
elif algo_choice ==2:
    choice = "rrt*"
else: 
    print("Please enter a valid algorithm number (1 or 2)")  

local_planner_choice = 1        # 1: dubins_curves, 2: reeds_shepp_curves (Reeds_shepp_curves under development)

"""
    Initalize parameters to run a simulation
"""
dt = 0.01 # the simulation time step
time_initial = time.time()

""" MAP SELECTION """
print("Select a map: ")
# print("1. default \n2. maze \n3. square_scatter\nEnter the map number: ")
map_choice = int( input("1. default \n2. maze \n3. square_scatter\nEnter the map number: "))
if map_choice == 1:
    obstacleFile='blank.csv'
elif map_choice == 2: 
    obstacleFile='maze.csv'
elif map_choice == 3:
    obstacleFile='square_scatter.csv'
else: 
    print("Please enter a valid map number (1,2 or 3)") 
""""""""""""""""""""""""""""""
doExport = False # export the simulation?
obstaclespace = []
ittr = 0 # keep track of simulation iterations
maxIttr = 20000  #how many time steps we want to simulate
globalTime = 0  # simuation time
reachedGoals = False # have all agents reached their goals
calibration = [] # to confirm directions of x and y axes
calib = 1
graduation = 2 # least count of grid
gridx = []
gridxtxt = []
gridy = []
gridytxt = []
theta_cost = 3
counter = 0
par_id = 0
#
goal_flag = 0
flag =0

count = 0
limit = 500
final_configurations = pygame.sprite.Group()
vertex_sprite_group = pygame.sprite.Group()
vertex_sprite_group_B = pygame.sprite.Group()
vertex_sprite_group_P = pygame.sprite.Group()
obstacle_space_sprite_group = pygame.sprite.Group()
grid_sprite_group = pygame.sprite.Group()
edge_sprite_group = pygame.sprite.Group()
old_time = 0

""" RRT* parameters
"""
edge_time = 4
interval_time = 1
goal_radius = 10            ## Goal region
local_planner_threshold = 1
sampling_limit = 3000      ## Sampling limit
fps = 5
# dubins curves parameters
dubins_radius_rrt = 4
dubins_resolution = 1
# robot dimensions
robot_width = 3
robot_height = 1
robot_radius = max(robot_width, robot_height)/2.
# Visualization options
enable_edges = 0
enable_intermediate_vertices = 1
enable_goal_path_vertices = 1
# nearest neighbor method
gam = 2000      ## tunable factor to control nearest neighbor radius
eta = 40        ## search radius cap
search_k = 15   ## number of nearest neighbors to be considered
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
""" Initialization """
RRT = Roadmap()
''' Start and goal point selection '''
start_vert = [-35 , 40 , 3.14*0/4]      #ORIGINAL
if obstacleFile=='maze.csv' or obstacleFile=='square_scatter.csv':
    q_goal = [40, -40, 3.14*6/4]      # Goal point for square_scatter and maze
if obstacleFile=='blank.csv':
    q_goal = [-40, 20, 3.14*3/2]        # Goal point for blank.csv (default) map
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
RRT.addVertex(start_vert, 0, 0, [], None)
reached = False
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"""
    Drawing parameters
"""
pixelsize = 800
framedelay = 30
drawVels = True
QUIT = False
paused = False
step = False
circles = []
velLines = []
gvLines = []
obsrect = []
vertices = []
edges = []
visualvertices = []
visualedges = []
global vertex_count
global visual_vertex_count
global edge_count
global visual_edge_count
global current_g

current_g = 0

vertex_count = 1
visual_vertex_count = 0
edge_count = 0
visual_edge_count = 0
vertex_rad = 0.5
vertex_rad2 = 0.2
vertex_rad3 = 0.4
visual_vertex_rad = 0.20

def readScenario(obstaclefile, scalex=1., scaley=1.):
    """
        Read a scenario from a file
    """

    fo = open(obstaclefile, 'r')
    lines = fo.readlines()
    fo.close()
    count = 0
    for line in lines:
        obstaclespace.append(BoxObstacle(line.split(','),count)) # create an agent and add it to the list
        count = count + 1

    # define the boundaries of the environment

    xmin_cand = [obs.x_min for obs in obstaclespace]
    xmax_cand = [obs.x_max for obs in obstaclespace]
    ymin_cand = [obs.y_min for obs in obstaclespace]
    ymax_cand = [obs.y_max for obs in obstaclespace]

    x_min =	min(xmin_cand)*scalex - 2.
    y_min =	min(ymin_cand)*scaley - 2.
    x_max =	max(xmax_cand)*scalex + 2.
    y_max =	max(ymax_cand)*scaley + 2.

    return x_min, x_max, y_min, y_max

class BoxObstacle(object):

    def __init__(self, points,id):
        self.points = points # the 4 vertices of the box
        self.id = id
        xs = np.asarray([float(points[0]),float(points[2]),float(points[4]),float(points[6])])
        ys = np.asarray([float(points[1]),float(points[3]),float(points[5]),float(points[7])])*(1)

        # for p in self.points:
        #     xs.append(p[0])
        #     ys.append(p[1])

        #AABB representation
        self.x_min = min(xs)
        self.x_max = max(xs)
        self.y_min = min(ys)
        self.y_max = max(ys)

        self.width = self.x_max - self.x_min
        self.height = self.y_max - self.y_min

world_xmin, world_xmax, world_ymin, world_ymax = readScenario(obstacleFile)
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize/world_width

q_range = [50.0,50.0,0]
robot_dim = [4.0,1.0]

# Import pygame.locals for easier access to key coordinates
# Updated to conform to flake8 and black standards
from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)
BLACK = (0,0,0)
GRAY = (100, 100, 100)
NAVYBLUE = ( 60, 60, 100)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = ( 0, 255, 0)
BLUE = ( 0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 128, 0)
PURPLE = (255, 0, 255)
CYAN = ( 0, 255, 255)
GREY = (220,220,220)


def nearest_neighbors(graph, q, max_dist): #Whats the point of this function? max_dist is set to 142. Every possible distance has to be less than this in x and y. You are comparing
    n_k = []                                # this to something which has cost of theta as well
    d_k = []
    for j in Roadmap.getVertices(graph):
        d = distance(j.q, q)
        if d <= max_dist:
            n_k.append(j)
            d_k.append(d)
    return n_k, d_k


def k_nearest(graph, q, k):
    n_k = []
    d_k = []
    list_n = []
    list_d = []
    ##
    N = len(graph.getVertices())
    max_dist = min(eta, gam * (np.log(N)) / N)
    # print("max_dist: ", max_dist, " | N: ", N)
    ##
    n_k, d_k = nearest_neighbors(graph, q, max_dist)

    for i in range(0,len(d_k)):
        max = d_k[i]
        loc = i
        for j in range(i,len(d_k)):
            if d_k[j] > max:
                max  = d_k[j]
                loc = j
        temp1 = d_k[i]
        d_k[i] = d_k[loc]
        d_k[loc] = temp1

        temp1 = n_k[i]
        n_k[i] = n_k[loc]
        n_k[loc] = temp1

    Z = n_k
    sorted_d_k = d_k

    Z.reverse()
    sorted_d_k.reverse()

    i = 0
    while i < k and i < len(Z):
        list_n.append(Z[i])
        list_d.append(sorted_d_k[i])
        i += 1
    return list_n, list_d


def k_nearest_g(graph, q, max_dist, k):
    n_k = []
    list_n = []
    list_g = []
    n_k, d_k = nearest_neighbors(graph, q, max_dist)
    N = len(n_k)

    g_k = []
    temp1 = 0
    for i in range(0,len(n_k)):
        g_k.append(n_k[i].g)

    if len(n_k) < k: k = len(n_k)
    for i in range(0,len(g_k)):
        max = g_k[i]
        loc = i
        for j in range(i,len(g_k)):
            if g_k[j] > max:
                max  = g_k[j]
                loc = j
        temp1 = g_k[i]
        g_k[i] = g_k[loc]
        g_k[loc] = temp1

        temp1 = n_k[i]
        n_k[i] = n_k[loc]
        n_k[loc] = temp1

    Z = n_k
    sorted_g_k = g_k
    Z.reverse()
    sorted_g_k.reverse()
    i = 0
    while i < k:
        list_n.append(Z[i])
        list_g.append(sorted_g_k[i])
        i += 1
    return list_n


def distance (q1, q2):

    distan = sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)
    return distan

def collision(q):

    for obstacle in obstaclespace:
        cx_min = obstacle.x_min - robot_radius
        cx_max = obstacle.x_max + robot_radius
        cy_min = obstacle.y_min - robot_radius
        cy_max = obstacle.y_max + robot_radius

        if ((q[0] >= cx_min) and (q[0] <= cx_max)and (q[1] >= cy_min) and (q[1] <= cy_max)):
            return True
    else:
        return False


def draw_config(config, color, name):
    global world_scale, world_xmin, world_ymin, robot_width, robot_height, final_configurations
    points = getRobotPlacement(config, robot_width, robot_height)
    corners = [(world_scale*(x[0]-world_xmin), world_scale*(-x[1] - world_ymin))  for x in points]

    final_configuration = car_orientation(config,corners,color)
    final_configurations.add(final_configuration)

def draw_path(path):
    global screen
    global final_configurations
    #start_color = np.array([227, 74, 51])
    #end_color =  np.array([254, 232, 200])
    #start_color = np.array([67,162,202])
    #end_color =  np.array([224,255,219])
    start_color = np.array([0,255,0])
    end_color =  np.array([0,0,255])
    for i in range(len(path)):
        color = start_color + float(i)/float(len(path)) * (end_color - start_color)
        int_color = tuple(color.astype(int))
        #tk_rgb = "#%02x%02x%02x" % tuple(int(c) for c in color)
        draw_config(path[i].q, int_color, "path")
    #final_configurations.draw(screen)

    #final_configurations.update()
    #pygame.display.flip()

def draw_point_path(plot_path):
    for point in (plot_path):
        new_vertex = vertex_sprite_P(point)
        vertex_sprite_group_P.add(new_vertex)

# Initialize pygame
pygame.init()

# pygame functions/classes
class car_orientation(pygame.sprite.Sprite):
    def __init__(self,center, corners, color):
        super().__init__()
        cornerx = []
        cornery = []
        lst = []
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*10, world_scale*10), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        # pygame.gfxdraw.filled_circle(self.image, int(world_scale*2.5), int(world_scale*2.5), int(world_scale*1), color)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))
        adjust = np.asarray(self.rect.topleft)

        for l in range(0,len(corners)):
            cornerx.append(corners[l][0] - adjust[0])
            cornery.append(corners[l][1] - adjust[1])
            lst.append(tuple(np.asarray([cornerx[l],cornery[l]])))
        pygame.draw.polygon(self.image,color,lst)

class vertex_sprite(pygame.sprite.Sprite):
    def __init__(self,center):
        super().__init__()
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*2*vertex_rad, world_scale*2*vertex_rad), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        pygame.gfxdraw.filled_circle(self.image, int(world_scale*vertex_rad), int(world_scale*vertex_rad), int(world_scale*vertex_rad), BLACK)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))

class vertex_sprite_B(pygame.sprite.Sprite):
    def __init__(self,center):
        super().__init__()
        self.q = center
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*2*vertex_rad2, world_scale*2*vertex_rad2), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        pygame.gfxdraw.filled_circle(self.image, int(world_scale*vertex_rad2), int(world_scale*vertex_rad2), int(world_scale*vertex_rad2), BLACK)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))

class vertex_sprite_P(pygame.sprite.Sprite):
    def __init__(self,center):
        super().__init__()
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*2*vertex_rad3, world_scale*2*vertex_rad3), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        pygame.gfxdraw.filled_circle(self.image, int(world_scale*vertex_rad3), int(world_scale*vertex_rad3), int(world_scale*vertex_rad3), BLUE)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))

class obstacle_space_sprite(pygame.sprite.Sprite):
    def __init__(self,dimension,cordinates):
        super().__init__()
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface(dimension)
        self.image.fill((255,0, 0))
        self.rect = self.image.get_rect()
        self.rect.topleft = (int(world_scale*(obs.x_min - world_xmin)),int(world_scale*(-obs.y_max - world_ymin)))
        #screen.blit(surf, (int(world_scale*(obs.x_min - world_xmin)),int(world_scale*(-obs.y_max - world_ymin))))
        # pygame.draw.polygon(self.image,color,corners)
        #screen.blit(self.image, cordinates)

class grid_sprite(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*world_width,world_scale*world_height),pygame.SRCALPHA)
        self.rect = self.image.get_rect()
        #self.rect.topleft = (int(world_scale*(obs.x_min - world_xmin)),int(world_scale*(-obs.y_max - world_ymin)))
        #screen.blit(surf, (int(world_scale*(obs.x_min - world_xmin)),int(world_scale*(-obs.y_max - world_ymin))))
        # pygame.draw.polygon(self.image,color,corners)

        i = float((int(world_xmin/graduation))*graduation)          # converting float to int (converts eg 13.5 to 10.0)
        while (i < world_xmax):
            if i == 0:
                pygame.draw.line(self.image, BLACK, (world_scale*(i - world_xmin), world_scale*(0)), (world_scale*(i - world_xmin), world_scale*(world_ymax - world_ymin)),2)
            if (i%10 == 0):
                #gridxtxt.append(canvas.create_text(world_scale*(i - world_xmin),world_scale*(2 - world_ymin),text=i)
                pygame.draw.line(self.image, BLACK, (world_scale*(i - world_xmin), world_scale*(0)), (world_scale*(i - world_xmin), world_scale*(world_ymax - world_ymin)),1)
            else:
                pygame.draw.line(self.image, GREY, (world_scale*(i - world_xmin), world_scale*(0)), (world_scale*(i - world_xmin), world_scale*(world_ymax - world_ymin)),1)
            i = i + graduation

        i = float((int(world_ymin/graduation))*graduation)          # converting float to int (converts eg 13.5 to 10.0)
        while (i < world_ymax):
            if i == 0:
                pygame.draw.line(self.image, BLACK, (world_scale*(0), world_scale*(-i - world_ymin)), (world_scale*(world_xmax - world_xmin), world_scale*(-i - world_ymin)),2)

            if (i%10 ==0):
                pygame.draw.line(self.image, BLACK, (world_scale*(0), world_scale*(-i - world_ymin)), (world_scale*(world_xmax - world_xmin), world_scale*(-i - world_ymin)),1)

            else:
                pygame.draw.line(self.image, GREY, (world_scale*(0), world_scale*(-i - world_ymin)), (world_scale*(world_xmax - world_xmin), world_scale*(-i - world_ymin)),1)
            i = i + graduation
        #screen.blit(self.image,(0,0))

class edge_sprite(pygame.sprite.Sprite):
    def __init__(self,startvertaddr,endvertaddr):
        super().__init__()
        self.startvertaddr = startvertaddr
        self.endvertaddr = endvertaddr
        startvert = np.asarray(startvertaddr)
        endvert = np.asarray(endvertaddr)

        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*world_width,world_scale*world_height),pygame.SRCALPHA)
        self.rect = self.image.get_rect()
        # pygame.draw.polygon(self.image,color,corners)
        pygame.draw.line(self.image, BLACK, (int(world_scale*(startvert[0] - world_xmin)), int(world_scale*(-startvert[1] - world_ymin))), (int(world_scale*(endvert[0] - world_xmin)), int(world_scale*(-endvert[1] - world_ymin))),1)
        self.rect = self.image.get_rect()
        #screen.blit(self.image,(0,0))


new_vertex = vertex_sprite(start_vert)
vertex_sprite_group.add(new_vertex)
# Define constants for the screen width and height

# Create the screen object
# The size is determined by the constant SCREEN_WIDTH and SCREEN_HEIGHT
position = 10, 10
os.environ['SDL_VIDEO_WINDOW_POS'] = str(position[0]) + "," + str(position[1])
screen = pygame.display.set_mode((int(world_scale*world_width), int(world_scale*world_height)))
# Run until the user asks to quit
running = True
clock = pygame.time.Clock()
count = 0
screen.fill((255, 255, 255))


for obs in obstaclespace:
    obstacle = obstacle_space_sprite((int(world_scale*(obs.x_max - obs.x_min)), int(world_scale*(obs.y_max - obs.y_min))),(int(world_scale*(obs.x_min - world_xmin)),int(world_scale*(-obs.y_max - world_ymin))))
    obstacle_space_sprite_group.add(obstacle)



# intital setup
if calib == 1:
    grid = grid_sprite()
    grid_sprite_group.add(grid)


def screen_update():
    screen.fill(WHITE)
    mp = 20
    pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),20)
    pygame.draw.circle(screen,PURPLE,(int(world_scale*(q_goal[0] - world_xmin)), int(world_scale*(-q_goal[1] - world_ymin))), int(world_scale*goal_radius))

    vertex_sprite_group.draw(screen)
    vertex_sprite_group_B.draw(screen)
    vertex_sprite_group_P.draw(screen)
    obstacle_space_sprite_group.draw(screen)
    if len(grid_sprite_group) != 0:
        grid_sprite_group.draw(screen)
    edge_sprite_group.draw(screen)
    if len(final_configurations) != 0:
        final_configurations.draw(screen)
    pygame.display.flip()


def duplicate(RRT, vertex):
    for vert in RRT.getVertices():
        if round(vert.q[0],3) == np.around(vertex[0],decimals = 3) and round(vert.q[1],3) == np.around(vertex[1],decimals = 3) and round(vert.q[2],3) == np.around(vertex[2],decimals = 3):
            return True
    return False


# (Reeds_shepp_curves under development)
'''
# def rsp_local_planner(start, end, radius): 

#     rspath = ReedsSheppPath(start, end, radius)
#     rspath.calc_paths()
#     opt1, length = rspath.get_shortest_path()
#     path = generate_pts(start, opt1, radius)
#     o1, o2 = make_opt(opt1)

#     return path, length, o1, o2

# def generate_pts(start, opt1, r):

#     pts = []
#     xs, ys, phi = ReedsSheppPath.gen_path(start, opt1, r)
#     for i in range(len(xs)):
#         for j in range(len(xs[i])):
#             pts.append([xs[i][j],ys[i][j]])
#     print(pts)
#     return pts

# def make_opt(item):

#     l = [item[0][1],item[2][1],item[1][1]]
#     l2 = True if item[1][0] == 's' else False
#     return l,l2
'''

# Working steer function
def steer(start, end, radius):
    if local_planner_choice == 1:
        lp = Dubins(radius , dubins_resolution)
        # flag = 0
        path_list = lp.all_options(start , end)                                         # getting sorted list of all possible 6 dubins curves between start and end point
        opt_path = path_list[0]                                                         # choosing the shortest path that does not collide
        path = lp.generate_points( start , end , opt_path[1] , opt_path[2] )            # Generating intermediate path points
        # Collision check
        for point in path:
            if collision(point) == True:
                return [], [], [], [], False
                # flag += 1
                # break
        # if flag==0:
        return path, opt_path[0], opt_path[1], opt_path[2], True
    
    elif local_planner_choice == 2:
        rspath = ReedsSheppPath(start, end, radius)
        rspath.calc_paths()
        opt1, length = rspath.get_shortest_path()
        path = generate_pts(start, opt1, radius)
        if path: 
            o1, o2 = make_opt(opt1)
            # zip check / path check:

            # Collision check
            for point in path:
                if collision(point) == True:
                    return [], [], [], [], False
                    # flag += 1
                    # break
            # if flag==0:
            return path, length, o1, o2, True
        else: 
            return [], [], [], [], False

def intermediates(path, start):
    prev_point = start
    for point in path:
        # # making path points (vertices)
        if enable_intermediate_vertices == 1:
            new_vertex = vertex_sprite_B(point)
            vertex_sprite_group_B.add(new_vertex)
        # # making mini edges (commented because computationally expensive)
        if enable_edges == 1:
            edge = edge_sprite( prev_point , point )
            edge_sprite_group.add(edge)
        prev_point = point

def path_finder(RRT, q_goal, goal_radius, k):
    opt_arr = []
    lp = Dubins( dubins_radius_rrt , dubins_resolution )
    n_v = k_nearest_g(RRT, q_goal, goal_radius, 10)
    parent_list = []
    plot_path = []
    global reached
    reached = False
    if len(n_v) !=0:
        vert = n_v[0]
        parent_list.append(vert)
        opt_arr.append(vert.path)

        child = vert
        cost = 99999999
        while cost!=0:
            opt_arr.append(child.path)      # for visualizing path profile
            par_v = RRT.getVertices()[child.p]
            temp_list , _,_,_,_ = steer ( par_v.q, child.q, dubins_radius_rrt)
            for item in temp_list:
                plot_path.append(item)
            parent_list.append(par_v)
            cost = par_v.g
            child = par_v
        reached = True

        ##### for visualization of path profile (below code is under development)
        # if path_profile_data == 1:
        #     if choice == "rrt*":
        #         if count > sampling_limit -1:
        #             r_arr = []
        #             for item in opt_arr:
        #                 r_arr.append(dubins_radius_rrt)
        #             get_length_segments(r_arr, opt_arr)
        #     else:
        #         r_arr = []
        #         for item in opt_arr:
        #             r_arr.append(dubins_radius_rrt)
        #         get_length_segments(r_arr, opt_arr)
        #########################################

        if count >= sampling_limit-1:
            print("Path length: ", parent_list[0].g)
    return parent_list, plot_path

def plot_latest_path(final_configurations, parent_list, plot_path, enable_goal_path_vertices, vertex_sprite_group_P):
    if len(final_configurations) == 0:              # if path does not exist, draw path
        draw_path(parent_list)                      # draws triangular vehicles
        if enable_goal_path_vertices == 1:
            draw_point_path(plot_path)              # draws colored  path

    else:
        for pt in final_configurations:             # if path exists, erase path and then draw new optimized path
            pt.remove(final_configurations)         # remove the vehicle versions
        draw_path(parent_list)                      # draws triangular vehicles
        if enable_goal_path_vertices == 1:
            # remove old path points
            for pt in vertex_sprite_group_P:
                pt.remove(vertex_sprite_group_P)    # remove old colored path points
            
            draw_point_path(plot_path)              # draws new optimized colored path points

def adjust_edges_and_vertices(edge_sprite_group, vertex_sprite_group_B, temp_list):
    for edge in edge_sprite_group:
        for i in range(len(temp_list)):
            if edge.startvertaddr[0] == temp_list[i][0] and edge.startvertaddr[1] == temp_list[i][1] and edge.endvertaddr[0] == temp_list[i+1][0] and edge.endvertaddr[1] == temp_list[i+1][1]:
                edge.remove(edge_sprite_group)
    l0 = len(vertex_sprite_group_B)
    l1 = len(temp_list)
    for pt in vertex_sprite_group_B:
        for point in temp_list:
            if point[0] == pt.q[0] and point[1] == pt.q[1]:
                pt.remove(vertex_sprite_group_B)
    l2 = len(vertex_sprite_group_B)


def rewire(RRT, n1_k, q_new):
    for x_near in n1_k:
        path, length, opt1, opt2, status = steer(q_new.q, x_near.q, dubins_radius_rrt)
        if status== True:
            new_cost = q_new.g + length
            if new_cost < x_near.g:
                x_par = x_near.p
                x_near_parent = RRT.getVertices()[x_par]
                RRT.removeEdge( x_near_parent , x_near)
                lp = Dubins(dubins_radius_rrt , dubins_resolution)
                temp_list = lp.generate_points(x_near_parent.q, x_near.q, x_near.path, x_near.path_type)
                # adjust visual edges and vertices
                adjust_edges_and_vertices(edge_sprite_group, vertex_sprite_group_B, temp_list)
                x_near.g = new_cost
                x_near.p = q_new.id
                x_near.path = opt1
                x_near.path_type = opt2
                intermediates(path, q_new.q)

def choose_parent(RRT, nearest_vertex, random_conf):
    path, length, opt1, opt2, status = steer(nearest_vertex.q , random_conf, dubins_radius_rrt)        # extract path points and other steer attributes
    if status == True:
        # choose parent
        c_min = nearest_vertex.g + length
        x_min = nearest_vertex
        d_min = length
        angles = opt1
        p_type = opt2
        par_id = nearest_vertex.id
        current_path = path


        # Find sorted list of nearest neighbors according to euclidean distance

        n1_k, d1_k = k_nearest(RRT, random_conf , search_k)
        for x_near in n1_k:
            path, length, opt1, opt2, status = steer(x_near.q , random_conf, dubins_radius_rrt)        # extract path points and other steer attributes
            if status == True:
                new_cost = x_near.g + length
                if new_cost < c_min:
                    current_path = path
                    x_min = x_near
                    c_min = new_cost
                    d_min = length
                    angles = opt1
                    p_type = opt2
                    par_id = x_near.id

        q_new = RRT.addVertex(random_conf, c_min, par_id, angles , p_type)
        RRT.addEdge(x_min, q_new, length, current_path)

        # Visualize intermediate points and edges
        intermediates(current_path, x_min.q)

        # Visualize new vertex in pygame
        new_vertex = vertex_sprite(random_conf)
        vertex_sprite_group.add(new_vertex)
        return q_new, n1_k, True
    return [], [], False

def rrt_star():
    y = random.uniform(-50,50)
    x = random.uniform(-50,50)
    theta = random.uniform(0,2*3.14)
    random_conf = np.asarray((x,y,theta))

    if collision(random_conf) == False:
        n_k, d_k = nearest_neighbors(RRT, random_conf, 142)
        if len(n_k) != 0:
            minpos = d_k.index(min(d_k))
            nearest_vertex = n_k[minpos]
            # Return path if it exists without collision
                #choose parent
            q_new, n1_k, status_cp = choose_parent(RRT, nearest_vertex, random_conf)
            if status_cp == True:
                # rewiring
                rewire(RRT, n1_k, q_new)

            ## Path Finding
            # find path and keep updating it.
            parent_list, plot_path = path_finder(RRT, q_goal, goal_radius, k=10)
            # plot path/updated path.
            plot_latest_path(final_configurations, parent_list, plot_path, enable_goal_path_vertices, vertex_sprite_group_P)
            if len(parent_list) != 0:
                return parent_list, plot_path, True
    return [], [], False

def rrt():
    y = random.uniform(-50,50)
    x = random.uniform(-50,50)
    theta = random.uniform(0,2*3.14)
    random_conf = np.asarray((x,y,theta))
    if collision(random_conf) == False:
        n_k, d_k = nearest_neighbors(RRT, random_conf, 20)
        if len(n_k) != 0:
            minpos = d_k.index(min(d_k))
            nearest_vertex = n_k[minpos]
            path, length, angles, p_type, status = steer(nearest_vertex.q , random_conf, dubins_radius_rrt)        # extract path points and other steer attributes
            if status == True:
                c_min = nearest_vertex.g + length
                par_id = nearest_vertex.id

                q_new = RRT.addVertex(random_conf, c_min, par_id, angles , p_type)
                RRT.addEdge(nearest_vertex, q_new, length, path)

                # Visualize intermediate points and edges
                intermediates(path, nearest_vertex.q)

                # Visualize new vertex in pygame
                new_vertex = vertex_sprite(random_conf)
                vertex_sprite_group.add(new_vertex)

            ## Path Finding

            # find path and keep updating it.
            parent_list, plot_path = path_finder(RRT, q_goal, goal_radius, k=10)
            # plot path/updated path.
            plot_latest_path(final_configurations, parent_list, plot_path, enable_goal_path_vertices, vertex_sprite_group_P)

            if len(parent_list) != 0:
                return parent_list, plot_path, True
    return [], [], False     


# execution of pygame code
while running:
    old_time = pygame.time.get_ticks()
    start_clock = pygame.time.Clock()
    count = count + 1
    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    while(flag == 0) and count <sampling_limit+1:
        if count%fps == 0:
            screen_update()
        count += 1
        if choice == "rrt":
            parent_list, plot_path, s1 = rrt()
            if s1 == True:
                if reached == True:
                    time_path = time.time()
                    screen_update()
                    flag = 1                            # Uncomment if you want to stop the code at the first path
                    if count >= sampling_limit-1:
                        print("Time taken to generate path: ", time_path - time_initial)
                        print("Number of RRT sampling iterations completed: ", count)

        if choice == "rrt*":
            parent_list, plot_path, s1 = rrt_star()
            if s1 == True:
                if reached == True:
                    if count==sampling_limit:
                        time_path = time.time()
                        print("Time taken to generate path: ", time_path - time_initial)
                        print("Number of RRT* sampling iterations completed: ", count)
pygame.quit()