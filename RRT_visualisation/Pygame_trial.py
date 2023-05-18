# -*- coding: utf-8 -*-
"""
Created on Sat Apr 15 11:27:30 2023

@author: SHREYASH
"""

import pygame
import random

# Initialize Pygame
pygame.init()

# Set the dimensions of the screen
screen_width = 640
screen_height = 480
screen = pygame.display.set_mode((screen_width, screen_height))

# Set the title of the window
pygame.display.set_caption("Shapes")

# Create a clock object to regulate the frame rate
clock = pygame.time.Clock()

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Define the main game loop
running = True
while running:

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)

    # Draw a rectangle
    rect_width = 100
    rect_height = 50
    rect_x = random.randint(0, screen_width - rect_width)
    rect_y = random.randint(0, screen_height - rect_height)
    pygame.draw.rect(screen, RED, (rect_x, rect_y, rect_width, rect_height))

    # Draw a circle
    circle_radius = 25
    circle_x = random.randint(circle_radius, screen_width - circle_radius)
    circle_y = random.randint(circle_radius, screen_height - circle_radius)
    pygame.draw.circle(screen, GREEN, (circle_x, circle_y), circle_radius)

    # Update the display
    pygame.display.update()

    # Regulate the frame rate
    clock.tick(10)

# Quit Pygame
pygame.quit()
