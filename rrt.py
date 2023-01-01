#Karnaa Mistry CS560 rrt.py

import numpy as np
from sampler import sample
from tree import Tree
from collision import isCollisionFree

def rrt(robot, obstacles, start, goal, iter_n):
    T = Tree(robot, obstacles, start, goal)
    for i in range(iter_n):
        qrand = sample()
        
        
        
       # if not isCollisionFree(robot, qrand, obstacles):
        #    continue

        qnearest = T.nearest(qrand)
        qnew = T.steer(qnearest, qrand, 1)
        #qnew = sample()
        #print('iter ',i,qnew)
        if not isCollisionFree(robot, qnew, obstacles):
            continue
        qnear = T.nearest(qnew)
        T.extend(qnear, qnew)
    qneargoal = T.nearest(goal)
    T.extend(qneargoal, goal)
    if T.exists(goal):
        return T.traceback(goal)
    else:
        return T.traceback(qneargoal)

#helper rrt method that returns the found path AND the tree, & if it succeeded
def rrt_tree(robot, obstacles, start, goal, iter_n):
    T = Tree(robot, obstacles, start, goal)
    for i in range(iter_n):
        qrand = sample()
        
        #if not isCollisionFree(robot, qrand, obstacles):
       #     continue

        if i%100 == 0:
            print('iter', i)
        qnearest = T.nearest(qrand)
        qnew = T.steer(qnearest, qrand, 1)
        #qnew = sample()
        #print('iter ',i,qnew)
        if not isCollisionFree(robot, qnew, obstacles):
            continue
        qnear = T.nearest(qnew)
        T.extend(qnear, qnew)
    qneargoal = T.nearest(goal)
    T.extend(qneargoal, goal)
    if T.exists(goal):
        return [T.traceback(goal),T,True,T.get_cost(goal)]
    else:
        return [T.traceback(qneargoal),T,False, T.get_cost(qneargoal)]


#helper distance method
def distance(p1, p2):
    p = np.asarray(p1)
    q = np.asarray(p2)
    return np.linalg.norm(p - q)