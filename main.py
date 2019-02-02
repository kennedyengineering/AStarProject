import pygame
from pygame.locals import *
#from queue import Queue
import time

width, height = 400, 300


class AStar: #A-Star algorithm class, for 4 way graph traversal
    def __init__(self, graph):
        self.finished = False
        self.openSet = []
        self.closedSet = []
        self.graph = graph
        self.openSet.append(graph.startNode)

    def manhattan(self, P, Q): #calculates manhattan AKA taxicab distance between two dimensional vectors P and Q
        return abs(P[0]-Q[1]) + abs(P[1]-Q[1])

    def cost(self, node): #calculate the travel cost of the current node to the start node
        total = 0
        parentNode = node.parent
        while parentNode != self.graph.startNode:
            total += parentNode.weight
            parentNode = parentNode.parent
        return total

    def evaluate(self, node): #evaluation function for finding the best node to expand
        return self.cost(node) + self.heuristic(node)

    def heuristic(self, node): #heuristic function, estimated minimum cost from start to goal
        return self.manhattan(node.coordinates, self.graph.goalNode.coordinates)

    def magic(self): #algorithm main loop, run once
        while not self.finished:
            bestNode = None
            evaluations = []
            for node in self.openSet:
                evaluation = self.evaluate(node)
                evaluations.append(evaluation)

            if len(evaluations) == 0: #if no more nodes are available to examine there is no solution
                self.finished = True
            else:
                bestEvaluation = evaluations[0]
                for evaluation in evaluations:
                    if evaluation < bestEvaluation:
                        bestEvaluation = evaluation
                for node in self.openSet:
                    evaluation = self.evaluate(node)
                    if evaluation == bestEvaluation:
                        bestNode = node
                        break

                if bestNode == self.graph.goalNode:
                    self.finished = True
                else:
                    self.openSet.remove(bestNode)
                    self.closedSet.append(bestNode)

                    if bestNode.left != None:
                        if bestNode.left not in self.closedSet:
                            if bestNode.left not in self.openSet:
                                self.openSet.append(bestNode.left)
                                bestNode.left.parent = bestNode

                    if bestNode.right != None:
                        if bestNode.right not in self.closedSet:
                            if bestNode.right not in self.openSet:
                                self.openSet.append(bestNode.right)
                                bestNode.right.parent = bestNode

                    if bestNode.up != None:
                        if bestNode.up not in self.closedSet:
                            if bestNode.up not in self.openSet:
                                self.openSet.append(bestNode.up)
                                bestNode.up.parent = bestNode

                    if bestNode.down != None:
                        if bestNode.down not in self.closedSet:
                            if bestNode.down not in self.openSet:
                                self.openSet.append(bestNode.down)
                                bestNode.down.parent = bestNode

        if len(evaluations) != 0:
            parentNode = bestNode.parent
            while parentNode != self.graph.startNode:
                self.graph.highlightNode(parentNode.coordinates)
                parentNode = parentNode.parent


class Graph: #graph class emulates a real graph by handing and rendering nodes and their relations
    def __init__(self, numNodesX, numNodesY, nodeSize, surface, goalNode, startNode):
        self.surface = surface
        self.nodeSize = nodeSize
        self.numNodesX = numNodesX
        self.numNodesY = numNodesY
        self.nodeArray = []
        self.goalNode = goalNode
        self.startNode = startNode

        for i in range(0, numNodesY):
            newArrayX = []
            for ii in range(0, numNodesX):
                newNode = Node((ii,i))
                newArrayX.append(newNode)
            self.nodeArray.append(newArrayX)

        for i in range(0, numNodesY):
            for ii in range(0, numNodesX):
                node = self.nodeArray[i][ii]
                if node.coordinates == self.goalNode.coordinates:
                    self.nodeArray[i].pop(ii)
                    self.nodeArray[i].insert(ii, self.goalNode)
                    node = self.goalNode
                elif node.coordinates == self.startNode.coordinates:
                    self.nodeArray[i].pop(ii)
                    self.nodeArray[i].insert(ii, self.startNode)
                    node = self.startNode
                #print(node.coordinates)
                if ii-1 != -1:
                    node.left = self.nodeArray[i][ii-1]
                if ii+1 != numNodesX:
                    node.right = self.nodeArray[i][ii+1]
                if i-1 != -1:
                    node.up = self.nodeArray[i-1][ii]
                if i+1 != numNodesY:
                    node.down = self.nodeArray[i+1][ii]


    def highlightNode(self, coordinates): #highlight a specific node on the graph by its coordinates
        theNode = None
        searchDone = False
        for array in self.nodeArray:
            for node in array:
                if coordinates == node.coordinates:
                    theNode=node
                    searchDone = True
                if searchDone:
                    break
            if searchDone:
                break
        
        if not theNode:
            return
        else:
            try:
                #theNode.left.fill = True
                #theNode.right.fill = True
                #theNode.up.fill = True
                #theNode.down.fill = True
                theNode.fill = True
            except:
                pass


    def render(self): #draw the graph and nodes
        X, Y = 30,30 #offset from window edge

        for i in range(0, len(self.nodeArray)):
            for node in self.nodeArray[i]:
                if node == self.goalNode:
                    pygame.draw.rect(self.surface, (255, 0, 0), [X, Y, self.nodeSize, self.nodeSize])
                elif node == self.startNode:
                    pygame.draw.rect(self.surface, (0, 255, 0), [X, Y, self.nodeSize, self.nodeSize])
                elif node.isWall:
                    pygame.draw.rect(self.surface, (100, 100, 100), [X, Y, self.nodeSize, self.nodeSize])
                elif node.fill:
                    pygame.draw.rect(self.surface, (255,255,255), [X, Y, self.nodeSize, self.nodeSize])
                else:
                    pygame.draw.rect(self.surface, (255,255,255), [X, Y, self.nodeSize, self.nodeSize], 1) 
                X += self.nodeSize
            Y += self.nodeSize
            X = 30


    def addWall(self, coordinates): #add an obstacle to the graph. Nodes do not interact with walls
        theNode = None
        searchDone = False
        for array in self.nodeArray:
            for node in array:
                if coordinates == node.coordinates:
                    theNode = node
                    searchDone = True
                if searchDone:
                    break
            if searchDone:
                break

        if not theNode:
            return
        else:
            theNode.isWall = True
            try:
                theNode.left.right = None
            except:
                pass
            try:
                theNode.right.left = None
            except:
                pass
            try:
                theNode.up.down = None
            except:
                pass
            try:
                theNode.down.up = None
            except:
                pass


class Node:
    def __init__(self, coordinates):
        self.left = None
        self.right = None
        self.up = None
        self.down = None
        self.coordinates = coordinates
        self.fill = False
        self.parent = None
        self.isWall = False
        self.weight = 1

    def getLeft(self):
        return self.left

    def getRight(self):
        return self.right

    def getUp(self):
        return self.up

    def getDown(self):
        return self.down

    def getCoordinates(self):
        return self.coordinates


class Goal(Node):
    def __init__(self, coordinates):
        Node.__init__(self, coordinates)


class Start(Node):
    def __init__(self, coordinates):
        Node.__init__(self, coordinates)
        self.parent = self

def main():
    pygame.init()
    clock = pygame.time.Clock()
    displaysurface = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Search Algorithm")

    #coordinates start at 0 E.X.: 1 --> 2, 7 --> 8
    wallCoordinates = [
                       (4,1),
                       (4,2),
                       (4,3),
                       (4,4),
                       (4,5),
                       (4,6),
                       (4,7)]
    numNodesX = 10
    numNodesY = 8
    goal = Goal((2,3))
    start = Start((6,7))
    graph = Graph(numNodesX,numNodesY,10,displaysurface, goal, start)
    for wall in wallCoordinates:
        graph.addWall(wall)
    #graph.highlightNode((0,0))

    astar = AStar(graph)
    #print(astar.heuristic(start))
    astar.magic()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()

        graph.render()
        pygame.display.update()
        clock.tick(30)

main()
