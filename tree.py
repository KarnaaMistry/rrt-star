#Karnaa Mistry CS560 tree.py

import numpy as np

from collision import isCollisionFree

class Tree:
    
    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        self.root = Node(start)
        self.nodes = [self.root]
        
    def add(self, point1, point2):
        if point1 == point2:
            return None
        nodes = self.nodes
        for n in nodes:
            if n.data == point1:
                child = Node(point2)
                child.parent = n
                n.children.append(child)
                self.nodes.append(child)
        return point2
        
    def exists(self, point):
        nodes = self.nodes
        for n in nodes:
            if n.data == point:
                return True
        return False
        
    def parent(self, point):
        nodes = self.nodes
        for n in nodes:
            if n.data == point:
                return n.parent.data
        return None
    
    def nearest(self, point):
        nodes = self.nodes
        p1 = np.asarray(point)
        mindist = np.linalg.norm(p1 - np.asarray(self.root.data))
        near = self.root
        for n in nodes:
            p2 = np.asarray(n.data)
            if np.linalg.norm(p1 - p2) < mindist:
                mindist = np.linalg.norm(p1 - p2)
                near = n
        return near.data
    
    def extend(self, point1, point2):
        p1 = np.asarray(point1)
        p2 = np.asarray(point2)
        #print("want to go to ", p2, "from", p1)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        d = 100 #DISCRETIZATION PARAMETER
        stepx = dx/d
        stepy = dy/d
        #print(stepx)
        prev = point1
        for i in range(d):
            newpoint = (prev[0]+stepx, prev[1]+stepy)
            if not isCollisionFree(self.robot, newpoint, self.obstacles):
                #print("collision at ", newpoint, "iter", i)
                return self.add(point1, prev)
                
            prev = newpoint
            
        #print("collision-free extension")
        
        return self.add(point1, point2)
        
    #traces back a path of points in the tree
    def traceback(self, point):
        path = []
        ptr = None
        for n in self.nodes:
            if (n.data == point):
                ptr = n
        while ptr.data != self.root.data:
            path.insert(0, ptr.data)
            ptr = ptr.parent
        path.insert(0, self.root.data)
        
        return path
    
    
    def get_cost(self, point):
        cost = 0
        ptr = None
        for n in self.nodes:
            if (n.data == point):
                ptr = n
        while ptr.data != self.root.data:
            parent = np.asarray(ptr.parent.data)
            current = np.asarray(ptr.data)
            cost = cost + np.linalg.norm(parent - current)
            ptr = ptr.parent
        
        return cost
            
    
    def rewire(self, point, r):
        xnew = point
        Xnear = self.near(point, r)
        xnewnode = None
        for n in self.nodes:
            if n.data == xnew:
                xnewnode = n
                break
        
        for xnear in Xnear:
            if (self.isObstacleFree(xnew, xnear) and self.get_cost(xnew) + distance(xnew, xnear) 
                < self.get_cost(xnear)):
                for n in self.nodes:
                    if n.data == xnear:
                        oldparentdata = n.parent.data
                            #print(n.data,'s parent: ',oldparent, 'became', j.data)
                            #change the parent, remove wherever it appears as someone's child
                        for p in self.nodes:
                            if oldparentdata == p.data:
                                for c in p.children:
                                    if c.data == xnear:
                                        p.children.remove(c)
                                        break
                        n.parent = xnewnode
                        xnewnode.children.append(n)
                        return
        
    
    #helper method to determine if two points are separated by an obstacle
    def isObstacleFree(self, point1, point2):
        #print('calling isObstacleFree for ', point1, point2)
        p1 = np.asarray(point1)
        p2 = np.asarray(point2)

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        d = 200 #DISCRETIZATION PARAMETER
        stepx = dx/d
        stepy = dy/d

        prev = point1
        for i in range(d):
            newpoint = (prev[0]+stepx, prev[1]+stepy)
            if not isCollisionFree(self.robot, newpoint, self.obstacles):
                return False
            prev = newpoint
        return True
    
    #finds neighborhood of a point in the tree within radius r
    def near(self, point, r):
        #print('calling near for ', point)
        #if not self.exists(point):
        #    return None
        neighborhood = []
        for n in self.nodes:
            #print('inspecting',n.data)
            q = np.asarray(n.data)
            p = np.asarray(point)
            if (np.linalg.norm(p - q) <= r) and self.isObstacleFree(n.data, point):
                neighborhood.append(n)
        data = []
        for d in neighborhood:
            data.append(d.data)
        return data
    
    #steering algorithm from the paper
    def steer(self, point1, point2, eta):
        if distance(point1, point2) <= eta:
            return point2
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        dist = distance(point1, point2)
        return (point1[0]+eta/dist*dx, point1[1]+eta/dist*dy)
        

class Node:
    def __init__(self, data):
        self.data = data
        self.parent = None
        self.children = []

        
        
#helper distance method
def distance(p1, p2):
    p = np.asarray(p1)
    q = np.asarray(p2)
    return np.linalg.norm(p - q)
        





        