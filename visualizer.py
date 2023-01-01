#Karnaa Mistry CS560 visualizer.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from matplotlib.collections import PatchCollection,LineCollection

from file_parse import parse_problem
from sampler import sample
from collision import isCollisionFree
from tree import Tree,Node
from rrt import rrt,rrt_tree
from rrt_star import rrt_star,rrt_star_tree
    

def visualize_problem(robot, obstacles, start, goal):
    patches = []
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
    startshape = []
    sx,sy = start[0], start[1]
    gx,gy = goal[0], goal[1]
    goalshape = []
    for r in range(len(robot)):
        startshape.append((robot[r][0] + sx, robot[r][1] + sy))
        goalshape.append((robot[r][0] + gx, robot[r][1] + gy))
    startshape = np.array(startshape, dtype=object)
    goalshape = np.array(goalshape, dtype=object)
    s1 = Polygon(startshape, color = 'crimson')
    g1 = Polygon(goalshape, color = 'green')
    patches.append(s1)
    patches.append(g1)
    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    ax.add_collection(p)

    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    

def visualize_points(points, robot, obstacles, start, goal):
    patches = []
    
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
    startshape = []
    sx,sy = start[0], start[1]
    gx,gy = goal[0], goal[1]
    goalshape = []
    for r in range(len(robot)):
        startshape.append((robot[r][0] + sx, robot[r][1] + sy))
        goalshape.append((robot[r][0] + gx, robot[r][1] + gy))
    startshape = np.array(startshape, dtype=object)
    goalshape = np.array(goalshape, dtype=object)
    s1 = Polygon(startshape, color = 'crimson')
    g1 = Polygon(goalshape, color = 'green')
    patches.append(s1)
    patches.append(g1)
    
    for x in range(len(points)):
        ghost = []
        for r in range(len(robot)):
            ghost.append((robot[r][0] + points[x][0], robot[r][1] + points[x][1]))
        c = len(points)-1
        if c == 0:
            g = Polygon(ghost, color = (240/255,40/255,220/255))
        else:
            g = Polygon(ghost, color = (-1/c*x+1, 0.85/c*x, 0.3/c*x+0.7))
        patches.append(g)

    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    ax.add_collection(p)

    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    
def visualize_path(robot, obstacles, path):
    patches = []
    lines = []
    linecolors = []
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
    
    for x in range(len(path)):
        ghost = []
        for r in range(len(robot)):
            ghost.append((robot[r][0] + path[x][0], robot[r][1] + path[x][1]))
        c = len(path)-1
        if c == 0:
            g = Polygon(ghost, color = (240/255,40/255,220/255))
        else:
            g = Polygon(ghost, color = (-1/c*x+1, 0.85/c*x, 0.3/c*x+0.7))
        if (x < len(path)-1):
            q = [(path[x][0],path[x][1]), (path[x+1][0],path[x+1][1])]
            linecolors.append((-1/c*(x+1)+1, 0.85/c*(x+1), 0.3/c*(x+1)+0.7))
            lines.append(q)
        patches.append(g)
        
    l = LineCollection(lines, linewidth = 3, colors = linecolors)
    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    ax.add_collection(p)
    ax.add_collection(l)

    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    
    
def visualize_configuration(robot, obstacles, start, goal):
    patches = []
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
    
    
    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    ax.add_collection(p)
    
    d = 125 # DISCRETIZATION FACTOR
    
    minx = 10
    maxx = 0
    miny = 10
    maxy = 0
    for r in robot:
        if r[0] < minx:
            minx = r[0]
        if r[0] > maxx:
            maxx = r[0]
        if r[1] < miny:
            miny = r[1]
        if r[1] > maxy:
            maxy = r[1]

    ax.axvspan(0,-minx,color='gray')
    ax.axvspan(10-maxx,10,color='gray')
    ax.axhspan(0,-miny,color='gray')
    ax.axhspan(10-maxy,10,color='gray')
    
    for i in np.arange(minx,10-maxx,10/d):
        for j in np.arange(miny,10-maxy,10/d):
            if not isCollisionFree(robot, (i,j), obstacles):
                plt.scatter(i,j,marker=',',s=25,color='gray')
                
    plt.scatter(x=start[0],y=start[1],c='crimson',s=50)
    plt.scatter(x=goal[0],y=goal[1],c='green',s=50)
    
    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    
    
    
def visualize_rrt(robot, obstacles, start, goal, iter_n):
    Tresult = rrt_tree(robot, obstacles, start, goal, iter_n)
    patches = []
    lines = []
    linecolors = []
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
       
    patches.append(Circle(goal, radius = 0.1, color = 'green'))
       
    d = 125 # DISCRETIZATION FACTOR
    
    blines = []
    for n in Tresult[1].nodes:
        patches.append(Circle(n.data, radius = 0.08, color = 'black'))
        for c in n.children:
            blines.append([n.data,c.data])
        
    path = Tresult[0]
                
    for x in range(len(path)):
        c = len(path)-1
        if c == 0:
            g = Circle(path[x][0], radius = 0.1, color = (240/255,40/255,220/255))
        else:
            g = Circle((path[x][0], path[x][1]), radius = 0.1, color = (-1/c*x+1, 0.85/c*x, 0.3/c*x+0.7))
        
        if (x < len(path)-1):
            q = [(path[x][0],path[x][1]), (path[x+1][0],path[x+1][1])]
            linecolors.append((-1/c*(x+1)+1, 0.85/c*(x+1), 0.3/c*(x+1)+0.7))
            lines.append(q)
        patches.append(g)
    
    b = LineCollection(blines, linewidth = 2, colors = 'black')
    l = LineCollection(lines, linewidth = 3, colors = linecolors)
    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    
    
    minx = 10
    maxx = 0
    miny = 10
    maxy = 0
    for r in robot:
        if r[0] < minx:
            minx = r[0]
        if r[0] > maxx:
            maxx = r[0]
        if r[1] < miny:
            miny = r[1]
        if r[1] > maxy:
            maxy = r[1]

    ax.axvspan(0,-minx,color='gray')
    ax.axvspan(10-maxx,10,color='gray')
    ax.axhspan(0,-miny,color='gray')
    ax.axhspan(10-maxy,10,color='gray')
    
    for i in np.arange(minx,10-maxx,10/d):
        for j in np.arange(miny,10-maxy,10/d):
            if not isCollisionFree(robot, (i,j), obstacles):
                plt.scatter(i,j,marker=',',s=25,color='gray')
     
    ax.add_collection(b)
    ax.add_collection(l)
    ax.add_collection(p)
    

    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    

    
    
def visualize_rrt_star(robot, obstacles, start, goal, iter_n):
    Tresult = rrt_star_tree(robot, obstacles, start, goal, iter_n)
    patches = []
    lines = []
    linecolors = []
    for o in obstacles:
       y = np.array(o)
       p1 = Polygon(y, color='gray')
       patches.append(p1)
       
    patches.append(Circle(goal, radius = 0.1, color = 'green'))
       
    d = 125 # DISCRETIZATION FACTOR
    
    blines = []
    for n in Tresult[1].nodes:
        patches.append(Circle(n.data, radius = 0.08, color = 'black'))
        for c in n.children:
            blines.append([n.data,c.data])
        
    path = Tresult[0]
    
                
    for x in range(len(path)):
        c = len(path)-1
        if c == 0:
            g = Circle(path[x][0], radius = 0.1, color = (240/255,40/255,220/255))
        else:
            g = Circle((path[x][0], path[x][1]), radius = 0.1, color = (-1/c*x+1, 0.85/c*x, 0.3/c*x+0.7))
        if (x < len(path)-1):
            q = [(path[x][0],path[x][1]), (path[x+1][0],path[x+1][1])]
            linecolors.append((-1/c*(x+1)+1, 0.85/c*(x+1), 0.3/c*(x+1)+0.7))
            lines.append(q)
        patches.append(g)
    
    b = LineCollection(blines, linewidth = 2, colors = 'black')
    l = LineCollection(lines, linewidth = 3, colors = linecolors)
    p = PatchCollection(patches, match_original = True)
    fig,ax = plt.subplots()
    
    
    minx = 10
    maxx = 0
    miny = 10
    maxy = 0
    for r in robot:
        if r[0] < minx:
            minx = r[0]
        if r[0] > maxx:
            maxx = r[0]
        if r[1] < miny:
            miny = r[1]
        if r[1] > maxy:
            maxy = r[1]

    ax.axvspan(0,-minx,color='gray')
    ax.axvspan(10-maxx,10,color='gray')
    ax.axhspan(0,-miny,color='gray')
    ax.axhspan(10-maxy,10,color='gray')
    
    for i in np.arange(minx,10-maxx,10/d):
        for j in np.arange(miny,10-maxy,10/d):
            if not isCollisionFree(robot, (i,j), obstacles):
                plt.scatter(i,j,marker=',',s=25,color='gray')
     
    ax.add_collection(b)
    ax.add_collection(l)
    ax.add_collection(p)
    

    plt.rcParams["figure.figsize"] = [10, 10]
    plt.grid(False)
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.xticks(np.arange(0,11))
    plt.yticks(np.arange(0,11))
    
    

# mainproblem = parse_problem('C:/Users/karna/.spyder-py3/kkm82/1/robot_env_01.txt',
#                           
#                             'C:/Users/karna/.spyder-py3/kkm82/1/probs_01.txt')

#bot = mainproblem[0]
#obs = mainproblem[1]
#strt = mainproblem[2][0][0]
#gol = mainproblem[2][0][1]












