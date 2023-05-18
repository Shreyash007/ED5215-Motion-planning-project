# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 22:35:37 2023

@author: SHREYASH
"""

import numpy as np
import matplotlib.pyplot as plt

# Define the boundaries of the workspace
x_min, x_max = -10, 10
y_min, y_max = -10, 10

# Define the start and goal positions
start = np.array([0, 0])
goal = np.array([8, 8])

# Define the step size and maximum number of iterations
step_size = 0.5
max_iter = 1000

# Define the RRT class
class RRT:
    def __init__(self, start, goal, x_min, x_max, y_min, y_max, step_size):
        self.start = start
        self.goal = goal
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.step_size = step_size
        self.vertices = [start]
        self.edges = []

    def generate_point(self):
        if np.random.random() < 0.1:
            point = self.goal
        else:
            point = np.random.uniform([self.x_min, self.y_min], [self.x_max, self.y_max])
        return point

    def find_nearest_vertex(self, point):
        distances = [np.linalg.norm(point - v) for v in self.vertices]
        nearest_vertex = self.vertices[np.argmin(distances)]
        return nearest_vertex
    
    def extend(self, point):
        nearest_vertex = self.find_nearest_vertex(point)
        direction = (point - nearest_vertex) / np.linalg.norm(point - nearest_vertex)
        new_vertex = nearest_vertex + self.step_size * direction
        if self.check_collision(new_vertex):
            self.vertices.append(new_vertex)
            self.edges.append((nearest_vertex, new_vertex))

    def check_collision(self, point):
        if point[0] < self.x_min or point[0] > self.x_max or point[1] < self.y_min or point[1] > self.y_max:
            return False
        for edge in self.edges:
            dist = np.linalg.norm(np.cross(edge[1] - edge[0], point - edge[0])) / np.linalg.norm(edge[1] - edge[0])
            if dist < self.step_size:
                return False
        return True

    def plan(self):
        for i in range(max_iter):
            point = self.generate_point()
            self.extend(point)
            if np.linalg.norm(self.vertices[-1] - self.goal) < self.step_size:
                self.vertices.append(self.goal)
                self.edges.append((self.vertices[-2], self.goal))
                break

    def plot(self):
        fig, ax = plt.subplots()
        for edge in self.edges:
            ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'k-')
        ax.plot([self.start[0], self.goal[0]], [self.start[1], self.goal[1]], 'ro')
        for vertex in self.vertices:
            ax.plot(vertex[0], vertex[1], 'bo')
        ax.set_xlim([self.x_min, self.x_max])
        ax.set_ylim([self.y_min, self.y_max])
        plt.show()

# Create a new RRT object and plan a path
rrt = RRT(start, goal, x_min, x_max, y_min, y_max, step_size)
rrt.plan()

# Visualize the result
rrt.plot()
