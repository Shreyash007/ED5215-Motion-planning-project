import pygame
from RRTBase import RRTGraph
from RRTBase import RRTMap
from RRTBase import smooth_path
import numpy as np
import time
import matplotlib.pyplot as plt


def main():
    dimensions =(512,512)
    start=(10,10)
    goal=(500,500)
    obsdim=150
    obsnum=4
    iteration=0
    num_iterations=1000
    t1=0
    goal_flag=True
    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeobs()
    #print(obstacles[0][0])
    map.drawMap(obstacles)

    t1=time.time()
    running =True
    trials=2
    while iteration < num_iterations and goal_flag:
            time.sleep(0.005)
            elapsed=time.time()-t1
            t1=time.time()
            #raise exception if timeout
            if elapsed > 20:
                print('timeout re-initiating the calculations')
                raise
    
            if iteration % 100 == 0:
                X, Y, Parent = graph.bias(goal)
                pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
                pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                 map.edgeThickness)
    
            else:
                X, Y, Parent = graph.expand()
                pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
                pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                 map.edgeThickness)
    
            if iteration % 5 == 0:
                pygame.display.update()
            iteration += 1
            
            
            if graph.path_to_goal() and goal_flag==True:
                print('Path is found in iteration:', iteration)
                print('average time per iteration (ms)', pygame.time.get_ticks() / (iteration))
                print('Number of nodes explored', len(X))
                goal_flag=False
                

    points=graph.getPathCoords()
    map.drawPath(graph.getPathCoords())
    for i in range(len(points)-1):
        pygame.draw.line(map.map, map.Green, (points[i]), (points[i+1]),
                         map.edgeThickness+2)
        pygame.display.update()
    
    path=smooth_path(points,obstacles)
    smoothed_points = path.conv_smooth_path()
    #Checking out new path
    path.new_path()

    # Extract x and y coordinates from points list using list comprehension
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    # Extract x and y coordinates from smoothed points using numpy array indexing
    
    smoothed_x = smoothed_points[:, 0]
    smoothed_y = smoothed_points[:, 1]
    #print(smoothed_points)
    map.drawPath2(smoothed_points)
    
    for i in range(len(smoothed_points)-1):
        pygame.draw.line(map.map, map.Red, (smoothed_points[i]), (smoothed_points[i+1]),
                         map.edgeThickness+2)
        pygame.display.update()
    # Plot the original and smoothed path
    plt.plot(x_coords, y_coords, 'bo-', label='Original Path')
    plt.plot(smoothed_x, smoothed_y, 'ro-', label='Smoothed Path')
    plt.legend()
    plt.show()
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(5000000)
    pygame.quit()
    return points,smoothed_points,obstacles
'''    
def smooth_path(points, window_size=3):
    # Convert points to a numpy array
    points = np.array(points)
    
    # Apply a moving average filter to x and y coordinates separately
    weights = np.repeat(1.0, window_size) / window_size
    smoothed_x = np.convolve(points[:,0], weights, mode='valid')
    smoothed_y = np.convolve(points[:,1], weights, mode='valid')
    
    # Combine smoothed x and y coordinates into a single array
    smoothed_points = np.vstack((smoothed_x, smoothed_y)).T
    
    return smoothed_points
'''

if __name__ == '__main__':
    result=False

    
    points,smoothed_points,obstacles=main()
    print(points)
    print(smoothed_points)
