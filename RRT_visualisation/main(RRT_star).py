# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 11:20:59 2023

@author: SHREYASH
"""

import pygame
import time

from RRT_Star import RRTGraph
from RRT_Star import RRTMap


# Select True for RRT Star or False for regular RRT
RRT_STAR = True

dimensions = (512, 512) # -y x
start = (50, 50)
goal = (500,500) # x -y
obsdim = 150
obsnum = 4
number_iterations= 1000
iteration=0

def main1():

    iteration=0
    pygame.init()
    map = RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obsnum, RRT_STAR)
    obstacles, obstaclesbbox = graph.makeobs()
    map.drawMapObs(obstacles, obstaclesbbox)

    X = []
    t1 = time.time()

    while (iteration < number_iterations):
        if X != []:
            map.undrawEdges(X, Y, Parents)
            

        time.sleep(0.05)
        elapsed = time.time() - t1
        t1 = time.time()

        if elapsed > 20:
            raise

        if iteration % 100 == 0:
            X, Y, Parents = graph.bias(goal)
            map.drawStuff(X, Y, Parents)


        else:
            X, Y, Parent = graph.expand()
            map.drawStuff(X, Y, Parents)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1

        if graph.reroutepathFlag:
            reroutedpath = graph.getPathCoords()
            map.drawPath(firstpath, (255, 255, 255), 8)
            map.drawPath(reroutedpath, (0, 255, 0), 8)
            firstpath = reroutedpath
            graph.reroutepathFlag = False

        graph.path_to_goal()
        if graph.goalFlag:
            firstpath = graph.getPathCoords()
            map.drawPath(firstpath, (255, 255, 0), 8)
            graph.goalFlag = False
            print('The first path is found in iteration:', iteration)
            print('average time per iteration (ms)', pygame.time.get_ticks() / (iteration))
            #graph.get_path_length()
            print('Number of nodes explored till first path', len(X))


        graph.change_path_to_goal()
        if graph.changeFlag:
            newpath = graph.getPathCoords()
            map.drawPath(firstpath, (255, 255, 255), 8)
            map.drawPath(newpath, (0, 255, 0), 8)
            firstpath = newpath
            graph.changeFlag = False
            graph.goalFlag = False


        pygame.event.wait(5)

        pygame.display.update()

        pygame.event.clear()
        
        if iteration == number_iterations:
            path = graph.getPathCoords()
            print('finised at iteration:', iteration)
            print('average time per iteration (ms)', pygame.time.get_ticks() / (iteration))
            #graph.get_path_length()
            print('Number of nodes explored at final path', len(X))
            for i in range(len(path)-1):
                pygame.draw.line(map.map, map.Red, (path[i]), (path[i+1]),
                                 map.edgeThickness+3)
            pygame.display.update()
            

    return path

if __name__ == '__main__':
    path = main1()
