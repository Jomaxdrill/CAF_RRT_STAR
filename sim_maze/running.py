import numpy as np
from caf_rrt_star import plan_caf_rrt_star
from solution import generate_path
from animation import create_animation
from maze import get_maze, get_maze_matrix
"""SET THE MAIN VARIABLES """
#Start and End of path
INITIAL= (500,1650,0)
GOAL = (5750,500,0)
# INITIAL= (75, 2826, 0)
# GOAL = (2915, 79, 0)
RADIUS_ROBOT = 220
width = 6000
height = 2000
border = 10
OPTION = 1
#*file is the name of an image of a maze in the maze folder. The background of the image must be white
file = 'map1.png'
given_map = None # LOaded image from file
matrix_map = None #binary matrix,  representation of image to be use in check collision
"""OPTION =1 maze 1 testing with 2D turtlebot3. Define width,height,border and radius robot.
OPTION = 2 maze 2 testing with 2D turtlebot3. Define width,height,border and radius robot.
OTION >= 3 maze from image
"""
if OPTION == 3:
    given_map = get_maze(f'mazes/{file}')
    matrix_map = get_maze_matrix(f'mazes/{file}')
    width_img, height_img = np.shape(matrix_map)
    width = width_img
    height = height_img
    border = 0
    RADIUS_ROBOT = 0
BORDER_TOTAL = border  + RADIUS_ROBOT
#Apply the algorithm
result = plan_caf_rrt_star(INITIAL, GOAL, OPTION, matrix_map, (BORDER_TOTAL, width, height))
if result is None:
    exit(0)
solution_path = generate_path((BORDER_TOTAL, width, height),OPTION, matrix_map , **result)
#for animation no radius robot needed
create_animation(INITIAL, GOAL, result, solution_path, given_map, OPTION, (border, width, height))
