# By Braedan Kennedy 2019

import pygame                                       # graphics library for rendering
from pygame.locals import *                         # utilities for the graphics library
from dpmap import *                                 # organized data, collected
import matplotlib.pyplot as plt
import numpy as np
import time

offset = 10                                         # offset from edge of screen, in pixels
nodeSize = 8                                        # size of each node, in pixels
width, height = (MAPSIZE[0]*nodeSize)+(2*offset), (MAPSIZE[1]*nodeSize)+(2*offset)              # width and height of the display window


class AStar: # A-Star algorithm class, for 4 way graph traversal
    def __init__(self, graph):
        self.finished = False                       # state identifier
        self.openSet = []                           # openSet, AKA the frontier, nodes to be evaluated and searched
        self.closedSet = []                         # closedSet, nodes that have been evaluated and searched
        self.graph = graph
        self.openSet.append(graph.startNode)        # start the search at the start node ---> goal node

    @staticmethod
    def manhattan(P, Q):  # calculates manhattan, AKA taxicab, distance between two dimensional vectors P and Q
        return abs(P[0]-Q[0]) + abs(P[1]-Q[1])

    def cost(self, node):  # calculate the travel cost of the current node to the start node AKA g(n)
        total = 0
        parentNode = node.parent
        while parentNode != self.graph.startNode:
            total += parentNode.weight
            parentNode = parentNode.parent
        return total

    def evaluate(self, node):  # evaluation function for finding the best node to expand in the openSet
        return self.cost(node) + self.heuristic(node)

    def heuristic(self, node):  # heuristic function, estimated minimum cost from start to goal
        return self.manhattan(node.coordinates, self.graph.goalNode.coordinates)

    def magic(self):  # algorithm main loop, run once
        while not self.finished:
            bestNode = None                         # node to be expanded, with the lowest estimated cost
            evaluations = []
            for node in self.openSet:               # evaluate each node in openSet
                evaluation = self.evaluate(node)
                evaluations.append(evaluation)

            if len(evaluations) == 0:               # if no more nodes are available to examine there is no solution
                self.finished = True
            else:                                   # if there are more nodes available then there is a solution, find it
                bestEvaluation = evaluations[0]
                for evaluation in evaluations:      # find best evaluation
                    if evaluation < bestEvaluation:
                        bestEvaluation = evaluation
                for node in self.openSet:           # match evaluation with first equivalent value node, AKA tie-breaker
                    evaluation = self.evaluate(node)
                    if evaluation == bestEvaluation:
                        bestNode = node
                        break

                if bestNode == self.graph.goalNode: # check if solution is found
                    self.finished = True
                else:
                    self.openSet.remove(bestNode)   # remove node from openSet
                    self.closedSet.append(bestNode) # add node to closedSet

                    if bestNode.left != None:       # investigate neighboring nodes for expansion, left neighbor
                        if bestNode.left not in self.closedSet:
                            if bestNode.left not in self.openSet:
                                self.openSet.append(bestNode.left)
                                bestNode.left.parent = bestNode

                    if bestNode.right != None:      # investigate neighboring nodes for expansion, right neighbor
                        if bestNode.right not in self.closedSet:
                            if bestNode.right not in self.openSet:
                                self.openSet.append(bestNode.right)
                                bestNode.right.parent = bestNode

                    if bestNode.up != None:         # investigate neighboring nodes for expansion, upper neighbor
                        if bestNode.up not in self.closedSet:
                            if bestNode.up not in self.openSet:
                                self.openSet.append(bestNode.up)
                                bestNode.up.parent = bestNode

                    if bestNode.down != None:       # investigate neighboring nodes for expansion, lower neighbor
                        if bestNode.down not in self.closedSet:
                            if bestNode.down not in self.openSet:
                                self.openSet.append(bestNode.down)
                                bestNode.down.parent = bestNode

        if len(evaluations) != 0:                   # render optimal path solution if available
            parentNode = bestNode.parent
            while parentNode != self.graph.startNode:
                self.graph.highlightNode(parentNode.coordinates)
                parentNode = parentNode.parent

            return self.cost(bestNode)


class Graph:  # graph class emulates a real graph by handing and rendering nodes and their relations with other nodes
    def __init__(self, numNodesX, numNodesY, nodeSize, surface, goalNode, startNode):
        self.surface = surface                      # rendering surface
        self.nodeSize = nodeSize                    # size in pixels, how large to draw each node as a square
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

    def highlightNode(self, coordinates, color = (255, 255, 255)):  # highlight a specific node on the graph identified by its coordinates
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
                theNode.fill = True
                theNode.fillColor = color
            except:
                pass

    def highlightNodeNode(self, node, color = (255, 255, 255)):
        try:
            node.fill = True
            node.fillColor = color
        except:
            pass

    def render(self):  # render the graph and nodes
        global offset
        X, Y = offset, offset                                # offset from window edge

        for i in range(0, len(self.nodeArray)):
            for node in self.nodeArray[i]:
                if node == self.goalNode:
                    pygame.draw.rect(self.surface, (255, 0, 0), [X, Y, self.nodeSize, self.nodeSize])
                elif node == self.startNode:
                    pygame.draw.rect(self.surface, (0, 255, 0), [X, Y, self.nodeSize, self.nodeSize])
                elif node.isWall:
                    #pass
                    pygame.draw.rect(self.surface, (100, 100, 100), [X, Y, self.nodeSize, self.nodeSize])
                elif node.fill:
                    pygame.draw.rect(self.surface, node.fillColor, [X, Y, self.nodeSize, self.nodeSize])
                else:
                    pass
                    #pygame.draw.rect(self.surface, (255,255,255), [X, Y, self.nodeSize, self.nodeSize], 1)
                X += self.nodeSize
            Y += self.nodeSize
            X = offset

    def addWall(self, coordinates):  # add an obstacle to the graph. Nodes do not interact with walls
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


class Node:  # node class. contains common attributes that all entities on the graph.
    def __init__(self, coordinates):
        self.left = None                            # node to the left relative to this node
        self.right = None                           # node to the right relative to this node
        self.up = None                              # node above this node
        self.down = None                            # node below this node
        self.coordinates = coordinates              # nodes coordinates on the graph
        self.fill = False                           # determine if node is to be drawn filled in or not
        self.fillColor = (255, 255, 255)
        self.parent = None                          # parent of the node
        self.isWall = False                         # determine if node is an obstacle or not
        self.weight = 1                             # travel weight from this node to its parent

    def getLeft(self):  # get method. return node to the left relative to this one
        return self.left

    def getRight(self):  # get. method. return node to the right relative to this one
        return self.right

    def getUp(self):  # get. method. return node above relative to this one
        return self.up

    def getDown(self):  # get. method. return node to the below relative to this one
        return self.down

    def getCoordinates(self):  # get method. return coordinates of the node
        return self.coordinates


class Goal(Node):  # goal node class. a different type of node representing the end point, goal, of the algorithm
    def __init__(self, coordinates):
        Node.__init__(self, coordinates)


class Start(Node):  # start node class. a different type of node representing the starting point of the algorithm
    def __init__(self, coordinates):
        Node.__init__(self, coordinates)
        self.parent = self


def demo():  # setup and execute the algorithm
    pygame.init()                                   # setup the graphics library for rendering
    clock = pygame.time.Clock()
    displaysurface = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Search Algorithm")

    # coordinates start at 0 E.X.: 1 --> 2, 7 --> 8
    wallCoordinates = [                             # define the locations of obstacles, coordinates
                       (4, 1), (5, 1), (6, 1), (7, 1), (8, 1), (9, 1), (10, 1),
                       (4, 2),
                       (4, 3),
                       (4, 4),
                       (4, 5),
                       (4, 6),
                       (4, 7)]
    numNodesX = 34                                  # define the number of nodes on the graph in the x direction
    numNodesY = 24                                  # define the number of nodes on the graph in the y direction
    goal = Goal((3,0))                              # define the goal node
    start = Start((10,7))                            # define the start node
    graph = Graph(numNodesX, numNodesY, 10, displaysurface, goal, start)               # define the graph
    for wall in wallCoordinates:                    # define the obstacles on the graph
        graph.addWall(wall)

    astar = AStar(graph)                            # use the graph for A-STAR algorithm
    cost = astar.magic()                                   # run the algorithm for a solution
    print(cost)

    while True:                                     # rendering loop, maintain the display window
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()

        graph.render()
        pygame.display.update()
        clock.tick(30)


def demo2():  # setup and execute the algorithm
    pygame.init()                                   # setup the graphics library for rendering
    clock = pygame.time.Clock()
    displaysurface = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Search Algorithm")

    background = pygame.image.load("dpmap grid overlay.png")    # prepare the background image for the display
    background = pygame.transform.scale(background, (width-(2*offset), height-(2*offset)))

    numNodesX = MAPSIZE[0]                             # define the number of nodes on the graph in the x direction
    numNodesY = MAPSIZE[1]                             # define the number of nodes on the graph in the y direction
    goal = Goal(CLASSES["MATH"])                       # define the goal node
    start = Start(CLASSES["ENGLISH"])                  # define the start node
    graph = Graph(numNodesX, numNodesY, nodeSize, displaysurface, goal, start)               # define the graph
    for building in DPMAP:                    # define the obstacles on the graph
        x = DPMAP[building][0]
        y = DPMAP[building][1]
        w = DPMAP[building][2]
        h = DPMAP[building][3]

        coordinateList = []
        for i in range(0, h):
            # i -> y
            coordinateList.append((x, y+i))
        for i in range(0, w):
            # i -> x
            coordinateList.append((x+i,y+h))
        for i in range(0, h):
            # i -> y
            coordinateList.append((x+w,y+i))
        for i in range(0, w):
            # i -> x
            coordinateList.append((x+i,y))
        coordinateList.append((x+w,y+h))

        for coordinate in coordinateList:
            graph.addWall(coordinate)

    astar = AStar(graph)                            # use the graph for A-STAR algorithm
    cost = astar.magic()                                   # run the algorithm for a solution
    print(cost)

    if False:                                       # select True/False to display nodes examined instead of optimal path
        for node in astar.openSet:
            graph.highlightNodeNode(node, color = (100, 100, 255))
        for node in astar.closedSet:
            graph.highlightNodeNode(node, color = (255, 100, 100))

    print(len(astar.openSet) + len(astar.closedSet))        # print out number of nodes examined

    while True:                                     # rendering loop, maintain the display window
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()

        displaysurface.blit(background, (offset, offset))
        graph.render()
        pygame.display.update()
        clock.tick(30)


def main():  # setup and execute the algorithm
    pygame.init()  # setup the graphics library for rendering
    displaysurface = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Search Algorithm")

    background = pygame.image.load("dpmap grid overlay.png")  # prepare the background image for the display
    background = pygame.transform.scale(background, (width - (2 * offset), height - (2 * offset)))

    numNodesX = MAPSIZE[0]                             # define the number of nodes on the graph in the x direction
    numNodesY = MAPSIZE[1]                             # define the number of nodes on the graph in the y direction

    trials = 8000
    nodesExamined = []
    distanceTraveled = []
    trial = 0

    startTime = time.time()

    while trial < trials:

        goodPoints = False
        while not goodPoints:
            check = False
            startX, startY = np.random.random_integers(0,numNodesX), np.random.random_integers(0,numNodesY)

            for building in DPMAP:
                x = DPMAP[building][0]
                y = DPMAP[building][1]
                w = DPMAP[building][2]
                h = DPMAP[building][3]

                if x <= startX <= x+w and y+h >= startY >= y:  # true if inside rectangle
                    check = False
                    break
                else:
                    check = True

            if check:
                goodPoints = True
        goodPoints = False
        while not goodPoints:
            check, check1 = False, False
            endX, endY = np.random.random_integers(0, numNodesX), np.random.random_integers(0, numNodesY)

            for building in DPMAP:
                x = DPMAP[building][0]
                y = DPMAP[building][1]
                w = DPMAP[building][2]
                h = DPMAP[building][3]

                if x <= endX <= x + w and y + h >= endY >= y:  # true if inside rectangle
                    check = False
                    break
                else:
                    check = True

            if endX==startX and endY==startY:
                check1 = False
            else:
                check1 = True

            if check and check1:
                goodPoints = True

        goal = Goal((endX, endY))                       # define the goal node
        start = Start((startX, startY))                  # define the start node
        graph = Graph(numNodesX, numNodesY, nodeSize, displaysurface, goal, start)               # define the graph
        for building in DPMAP:                    # define the obstacles on the graph
            x = DPMAP[building][0]
            y = DPMAP[building][1]
            w = DPMAP[building][2]
            h = DPMAP[building][3]

            coordinateList = []
            for i in range(w+1):
                for ii in range(h+1):
                    coordinateList.append((x+i,y+ii))

            '''
            coordinateList = []
            for i in range(0, h):
                # i -> y
                coordinateList.append((x, y+i))
            for i in range(0, w):
                # i -> x
                coordinateList.append((x+i,y+h))
            for i in range(0, h):
                # i -> y
                coordinateList.append((x+w,y+i))
            for i in range(0, w):
                # i -> x
                coordinateList.append((x+i,y))
            coordinateList.append((x+w,y+h))
            '''

            for coordinate in coordinateList:
                graph.addWall(coordinate)

        astar = AStar(graph)                            # use the graph for A-STAR algorithm
        cost = astar.magic()                                   # run the algorithm for a solution

        if cost != None:
            nodesExamined.append(len(astar.openSet) + len(astar.closedSet))
            distanceTraveled.append(cost)
            trial += 1
        else:
            print("retrying")

        print(trial)

        displaysurface.blit(background, (offset, offset))
        graph.render()
        pygame.display.update()

    plt.subplot(2, 1, 1)
    plt.scatter(nodesExamined, distanceTraveled)
    plt.title("Simulation results, " + str(trials) + " iterations")
    plt.xlabel("Quantity of nodes examined")
    plt.ylabel("Cost of optimal path")

    distanceTraveledMeters = []
    for cost in distanceTraveled:
        distanceTraveledMeters.append(cost*4)

    areaExamined = []
    for node in nodesExamined:
        areaExamined.append(node*16)

    plt.subplot(2, 1, 2)
    plt.scatter(areaExamined, distanceTraveledMeters)
    plt.xlabel("Area examined (M^2)")
    plt.ylabel("Distance (M)")

    endTime = time.time() - startTime
    print("Time: ", endTime, " seconds")

    plt.show()
    pygame.quit()
    exit()

# demo()                                              # start the demo
# demo2()                                             # start the second demo
main()                                               # run the main program
