import sys, random, math
import numpy as np
from math import sqrt,cos,sin,atan2

#constants
XDIM = 57.5
YDIM = 38.1
NUMNODES = 1000
stepSize = 1.16 
obstacleStartX = 27.0
obstacleStartY = 0.0
obstacleEndY = 20.0
path = []
branches = []
forward = 1
reverse = 2
left = 3
right = 4
dtheta = 0.23
distance = 1.16

def step_from_to(p1,p2):
    if dist(p1,p2) < stepSize:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + stepSize*cos(theta), p1[1] + stepSize*sin(theta)        
        
def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

class Node:
    x = 0
    y = 0
    def __init__(self,xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord

class Edge:
    vector = []
    x = 0
    y = 0
    def __init__(self, vec, xcoord, ycoord):
        self.vector = vec
        self.x = xcoord
        self.y = ycoord        
        
def ObstacleSpace(obsXDIM, obsYDIM):
    obs = []
    for i in range(obsXDIM):
        obs.append(Node(obstacleStartX+i, obstacleStartY))
        obs.append(Node(obstacleStartX+i, obstacleEndY))
    for j in range(obsYDIM):
        obs.append(Node(obstacleStartX, obstacleStartY+j))
    return obs
               
def checkIntersect(obstaclePos, predictedNode, nn):
    vec = [predictedNode[0]-nn.x, predictedNode[1]-nn.y]
    theta = atan2(predictedNode[1]-nn.y,predictedNode[0]-nn.x)
    vecSpace = []
    for m in range(9):
        vecSpace.append(Node(nn.x+(m/10)*stepSize*cos(theta), nn.y+(m/10)*stepSize*sin(theta)))
    for i in obstaclePos:
        for n in vecSpace:
            if i.x == n.x and i.y == n.y:
                return 0
    branches.append(Edge(vec, nn.x, nn.y))
    return 2

def radiusCheck(interpolatedNode, goal):
    return sqrt((interpolatedNode[0]-goal.x)*(interpolatedNode[0]-goal.x)+(interpolatedNode[1]-goal.y)*(interpolatedNode[1]-goal.y))

def rrt(start, goal, nodeFamily, obstacleSpace):
    for i in range(NUMNODES):
        rand = Node(random.random()*XDIM, random.random()*YDIM)
        nn = nodeFamily[0]
        for p in nodeFamily:
             if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
                nn = p        
        interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
        if checkIntersect(obstacleSpace, interpolatedNode, nn) > 1:
            nodeFamily.append(Node(interpolatedNode[0], interpolatedNode[1]))
        #if interpolatedNode[0] == goal.x and interpolatednode[1] == goal.y:
            #return nodeFamily 
        if  radiusCheck(interpolatedNode, goal) < 2:
            return nodeFamily
    return nodeFamily 

def FindPath(nodeFamily):
    extractedpaths = 0
    goalState = nodeFamily[-1]
    currentNode = [goalState.x, goalState.y]
    for i in reversed(branches):
        #print("Prev Node: ", prevNode)
        difference = list(np.array(currentNode) - np.array(i.vector))
        #print("Difference: ", difference)
        for m in reversed(branches):
            prevNode = [m.x, m.y]
            if difference == prevNode:
                currentNode = prevNode
                path.append(i)
                extractedpaths=extractedpaths+1
                break
    print(extractedpaths)

    
def main():
    nodes = []
    
    nodes.append(Node(11.4,10.2)) # Start in the corner
    start=nodes[0]
    goal=Node(36.5,11.4)
    obstacleSpace = ObstacleSpace(9, 20)
    nodeFamily = rrt(start, goal, nodes, obstacleSpace)
    
    reachedGoal = nodeFamily[-1]
    print(reachedGoal.x)
    print(reachedGoal.y)
    
    FindPath(nodeFamily)
    for i in path:
        print(i.vector),
        print(i.x),
        print(i.y)
     
    
if __name__ == '__main__':
    main()