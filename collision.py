#Karnaa Mistry CS560 collision.py

import numpy as np


def isCollisionFree(robot, point, obstacles):
    
    pts = [] #points of a config
    for r in range(len(robot)):
        pts.append((robot[r][0] + point[0], robot[r][1] + point[1]))
    for pts1 in pts: #boundary cases
        if pts1[0] < 0 or pts1[0] > 10 or pts1[1] < 0 or pts1[1] > 10:
            return False
    
    for pt in pts: #ray casting
        p1 = pt
        p2 = (10, pt[1])
        count = 0 #if odd, point is within an obstacle
        for o in obstacles:
            for b in range(len(o)-1):
                q1 = (o[b][0], o[b][1])
                q2 = (o[b+1][0], o[b+1][1])
                if intersect(p1, p2, q1, q2):
                    count = count+1
            if intersect(p1, p2, (o[len(o)-1][0], o[len(o)-1][1]),
                         (o[0][0], o[0][1])):
                count = count+1  
        if count%2 == 1:
            return False
        
    for o in obstacles: #ray casting if obstacle is contained in robot
        for pt in o:
            p1 = pt
            p2 = (10, pt[1])
            count = 0 #if odd, point is within an obstacle
            for b in range(len(pts)-1):
                q1 = (pts[b][0], pts[b][1])
                q2 = (pts[b+1][0], pts[b+1][1])
                if intersect(p1, p2, q1, q2):
                    count = count+1
            if intersect(p1, p2, (pts[len(pts)-1][0], pts[len(pts)-1][1]),
                         (pts[0][0], pts[0][1])):
                    count = count+1  
            if count%2 == 1:
                return False
        
    
    for t in range(len(pts)-1): #boundary collision checking
        p1 = (pts[t][0], pts[t][1])
        p2 = (pts[t+1][0], pts[t+1][1])
        
        for o in obstacles:
            for b in range(len(o)-1):
                q1 = (o[b][0], o[b][1])
                q2 = (o[b+1][0], o[b+1][1])
                if intersect(p1, p2, q1, q2):
                    return False
            if intersect(p1, p2, (o[len(o)-1][0], o[len(o)-1][1]),
                         (o[0][0], o[0][1])):
                return False  
            
    p1 = (pts[len(pts)-1][0], pts[len(pts)-1][1])
    p2 = (pts[0][0], pts[0][1])
    
    for o in obstacles:
            for b in range(len(o)-1):
                q1 = (o[b][0], o[b][1])
                q2 = (o[b+1][0], o[b+1][1])
                if intersect(p1, p2, q1, q2):
                    return False
            if intersect(p1, p2, (o[len(o)-1][0], o[len(o)-1][1]),
                         (o[0][0], o[0][1])):
                return False  
    
    return True
        


        
#helper intersect method, adapted from Boe line intersection algorithm
#all opposite orientations --> line segments separate point-pairs
def intersect(p1,p2,q1,q2):
    if (cc(p2,q1,q2) != cc(p1,q1,q2) and cc(p1,p2,q1) != cc(p1,p2,q2)):
        return True
#second helper method for line intersection (determines if slope AB < slope AC --> counter-clockwise)
def cc(a,b,c):
    ax,bx,cx = a[0],b[0],c[0]
    ay,by,cy = a[1],b[1],c[1]
    return ((by-ay)*(cx-ax) < (cy-ay)*(bx-ax))
    
    
    
    
    
    
    
    
    
