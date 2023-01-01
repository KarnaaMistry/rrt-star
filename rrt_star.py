#Karnaa Mistry CS560 rrt_star.py

import numpy as np
from sampler import sample
from tree import Tree
from collision import isCollisionFree
    
def rrt_star(robot, obstacles, start, goal, iter_n):
    T = Tree(robot, obstacles, start, goal)
    for i in range(iter_n):
        
        qrand = sample()

        qnearest = T.nearest(qrand)
        qnew = T.steer(qnearest, qrand, 1)
        #if i%10 == 0:
           # print('iter ',i,qnew)
        if not T.isObstacleFree(qnearest, qnew):

            continue
        Xnear = T.near(qnew, 2)
        xmin = qnearest
        cmin = T.get_cost(qnearest) + distance(qnearest, qnew)
        for xnear in Xnear:
            if T.isObstacleFree(xnear, qnew) and T.get_cost(xnear) + distance(xnear, qnew) < cmin:
                xmin = xnear
                cmin = T.get_cost(xnear) + distance(xnear,qnew)
        T.extend(xmin, qnew)
        T.rewire(qnew, 2)

    qneargoal = T.nearest(goal)
    T.extend(qneargoal, goal)
    if not T.exists(goal):
        return T.traceback(qneargoal)
    else:
        T.rewire(goal, 2)
        return T.traceback(goal)

#helper rrt method that returns the found path AND the tree
def rrt_star_tree(robot, obstacles, start, goal, iter_n):
    T = Tree(robot, obstacles, start, goal)
    for i in range(iter_n):
        
        qrand = sample()

        qnearest = T.nearest(qrand)
        qnew = T.steer(qnearest, qrand, 1)
        #if i%10 == 0:
        #    print('iter ',i,qnew)
        if not T.isObstacleFree(qnearest, qnew):
            continue
        Xnear = T.near(qnew, 2)
        xmin = qnearest
        cmin = T.get_cost(qnearest) + distance(qnearest, qnew)
        for xnear in Xnear:
            if T.isObstacleFree(xnear, qnew) and T.get_cost(xnear) + distance(xnear, qnew) < cmin:
                xmin = xnear
                cmin = T.get_cost(xnear) + distance(xnear,qnew)
        T.extend(xmin, qnew)
        T.rewire(qnew, 2)

    qneargoal = T.nearest(goal)
    T.extend(qneargoal, goal)

    if not T.exists(goal):
        return [T.traceback(qneargoal),T,False,T.get_cost(qneargoal)]
    else:
        T.rewire(goal, 2)
        return [T.traceback(goal),T,True, T.get_cost(goal)]


#helper distance method
def distance(p1, p2):
    p = np.asarray(p1)
    q = np.asarray(p2)
    return np.linalg.norm(p - q)




