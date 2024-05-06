import cv2
import math
import numpy as np

def coordinate_image(state, transf_image):
	"""
	This function takes a state as input and returns the corresponding row and column for an image
	Args:
		state (tuple): The state of the robot, as a tuple of its x and y coordinates.

	Returns:
		tuple: The row and column coordinates of the state in the transformed image.

	"""
	x_pos, y_pos = state
	row, col, _ = np.dot(transf_image, (x_pos, y_pos, 1))
	return int(row),int(col)
def distance(node_a, node_b):
	"""
	Returns the Euclidean distance between two nodes.

	Args:
		node_a (tuple): The first node.
		node_b (tuple): The second node.

	Returns:
		float: The Euclidean distance between the two nodes.

	"""
	substract_vector = get_vector(node_a, node_b)
	return round(math.sqrt(substract_vector[0]**2 + substract_vector[1]**2),2)

def get_vector(node_a, node_b):
	"""
	This function returns the vector from node_a to node_b.

	Args:
		node_a (tuple): The first node.
	"""
	return (node_b[0] - node_a[0], node_b[1] - node_a[1])

def get_maze(file):
	"""
	This function reads an image file and returns the corresponding maze.

	Parameters:
	file (str): The path to the image file containing the maze.

	Returns:
	numpy.ndarray: The maze image as a NumPy array. If the file is not provided or cannot be read, returns None.

	Example:
	>>> maze = get_maze('maze.png')
	>>> print(maze.shape)
	(500, 500, 3)
	"""
	if file:
		return cv2.imread(file)
	return None
def get_maze_matrix(file):
	"""
	This function reads an image file, converts it to grayscale, applies a binary threshold,
	and then converts the resulting binary image to a matrix representation of the maze.

	Parameters:
	file (str): The path to the image file containing the maze.

	Returns:
	numpy.ndarray: The maze matrix as a NumPy array. If the file is not provided or cannot be read, returns None.

	Example:
	>>> maze_matrix = get_maze_matrix('maze.png')
	>>> print(maze_matrix.shape)
	(500, 500)
	"""
	if file:
		maze = cv2.imread(file)
		maze_gray = cv2.cvtColor(maze, cv2.COLOR_BGR2GRAY)
		_, maze_binary = cv2.threshold(maze_gray, 240, 255, cv2.THRESH_BINARY)
		maze_matrix = np.where(maze_binary == 255, 1, 0)
		return maze_matrix
	return None
def interpolate_line(start_point, end_point, inter_points=30):
	"""
	This function calculates a set of points that lie on a straight line segment between two given points.
	The line is interpolated using linear interpolation.

	Parameters:
	start_point (tuple): A tuple representing the (x, y) coordinates of the start point of the line segment.
	end_point (tuple): A tuple representing the (x, y) coordinates of the end point of the line segment.
	inter_points (int, optional): The number of intermediate points to calculate between the start and end points.
		Defaults to 30.

	Returns:
	list: A list of tuples, where each tuple represents the (x, y) coordinates of an interpolated point.

	Example:
	>>> interpolate_line((0, 0), (10, 10), 5)
	[(0.0, 0.0), (2.5, 2.5), (5.0, 5.0), (7.5, 7.5), (10.0, 10.0)]
	"""
	points = []
	for idx in range(inter_points):
		t_idx = idx / (inter_points - 1)  # Calculate the parameter t
		x_point = start_point[0] + t_idx * (end_point[0] - start_point[0])  # Linear interpolation for x coordinate
		y_point = start_point[1] + t_idx * (end_point[1] - start_point[1])  # Linear interpolation for y coordinate
		points.append((x_point, y_point))
	return points

def collision(node, new_node, option, constraints, inter_points, matrix= None):
	"""
	This function checks if a collision occurs between a line segment defined by two nodes and obstacles.

	Parameters:
	node (tuple): The coordinates of the starting point of the line segment.
	new_node (tuple): The coordinates of the ending point of the line segment.
	option (int): The option to determine the obstacle space.
	constraints (tuple): The constraints for the obstacle space.
	inter_points (int): The number of intermediate points to interpolate between the start and end points.
	matrix (numpy.ndarray, optional): The matrix representation of the maze for image-based obstacle detection.

	Returns:
	bool: True if a collision occurs, False otherwise.

	"""
	hit = False
	line = interpolate_line(node, new_node, inter_points)
	for point in line:
		if check_in_obstacle(point, option, constraints, matrix):
			hit = True
			break
	return hit


def check_in_obstacle(state, option, constraints, matrix = None):
	"""
	This function checks if a given state is within the obstacle space based on the selected option.

	Parameters:
	state (tuple): The horizontal and vertical coordinates of the state.
	option (int): The option to determine the obstacle space.
	constraints (tuple): The constraints for the obstacle space.
	matrix (numpy.ndarray, optional): The matrix representation of the maze for image-based obstacle detection.

	Returns:
	bool: True if the state is within the obstacle space, False otherwise.

	Raises:
	ValueError: If the option is not 1, 2, or 3.

	"""
	#these options contemplate that the robot has a radius and a space defined by half plane equations
	if option == 1:
		return check_in_obstacle_one(state, constraints)
	elif option == 2:
		return check_in_obstacle_two(state, constraints)
	else:
		#default option contemplate a robot point for a complex maze from an image
		return check_in_obstacle_image(state, constraints, matrix)
def check_in_obstacle_image(state, constraints, matrix):
	"""
	Checks if a given state is within the obstacle space based on an image representation of the maze.

	Parameters:
	state (tuple): The horizontal and vertical coordinates of the state.
	constraints (tuple): The constraints for the obstacle space, containing the width and height of the space.
	matrix (numpy.ndarray): The matrix representation of the maze for image-based obstacle detection.

	Returns:
	bool: True if the state is within the obstacle space, False otherwise. If the matrix is None, it returns True.

	"""
	if matrix is None:
		print('There is no space to look for')
		return True
	_, WIDTH_SPACE, HEIGHT_SPACE = constraints
	if state[0] <= 0  or state[1] <= 0 or state[0] >= WIDTH_SPACE or state[1] >= HEIGHT_SPACE:
		return True
	TRANS_MATRIX = [ [0,-1, HEIGHT_SPACE],[1, 0, 0],[0, 0, 1] ] #from origin coord system to image coord system
	row, col = coordinate_image(state,TRANS_MATRIX)
	return  matrix[row][col]== 0

def check_in_obstacle_one(state, constraints):
	"""
	This function checks if a given state is within the obstacle space.

	Args:
		state (tuple): The horizontal and vertical coordinates of the state.
		border (int): The clearance of the obstacles.

	Returns:
		bool: True if the state is within the obstacle space, False otherwise.
	"""
	BORDER, WIDTH_SPACE, HEIGHT_SPACE = constraints
	sc = 1
	tl = BORDER / sc
	x_pos, y_pos = state
	x_pos = x_pos/sc
	y_pos = y_pos/sc
	# Check if the state is outside of the space
	if x_pos < 0 or y_pos < 0:
		#print('outside')
		return True
	if x_pos >= WIDTH_SPACE/sc or y_pos >= HEIGHT_SPACE/sc:
		#print('outside')
		return True
	#first obstacle
	in_obstacle_0 = (x_pos >= 1500/sc - tl) and (x_pos <= 1750/sc + tl) and (y_pos >= 1000/sc - tl) and (y_pos <= HEIGHT_SPACE/sc)
	if in_obstacle_0:
		#print(f'first obstacle')
		return True
	#second obstacle
	in_obstacle_1 = (x_pos >= 2500/sc - tl) and (x_pos <= 2750/sc + tl) and (y_pos/sc >= 0) and (y_pos <= 1000/sc + tl)
	if in_obstacle_1:
		#print(f'second obstacle')
		return True
	#third_obstacle- circle
	in_obstacle_2 = (x_pos - 4200/sc)**2 + (y_pos - 1200/sc)**2 <= (600/sc + tl)**2
	if in_obstacle_2:
		#print(f'third obstacle')
		return True
	#border wall 1
	walls_1 = np.zeros(3, dtype=bool)
	walls_1[0] = ( x_pos >= 0 and x_pos <= 1500/sc - tl ) and (y_pos >= HEIGHT_SPACE/sc - tl and y_pos <= HEIGHT_SPACE/sc)
	walls_1[1] = ( x_pos >= 0 and x_pos <= tl ) and (y_pos >= tl and y_pos <= HEIGHT_SPACE/sc - tl)
	walls_1[2] =  ( x_pos >= 0 and x_pos <= 2500/sc - tl ) and (y_pos >= 0 and  y_pos <= tl )
	in_obstacle_4 = any(walls_1)
	if in_obstacle_4:
		#print(f'walls left detected')
		return True
	#border wall 2
	walls_2 = np.zeros(3, dtype=bool)
	walls_2[0] = ( x_pos >= 1750/sc + tl and x_pos <= WIDTH_SPACE/sc ) and (y_pos >= HEIGHT_SPACE/sc - tl and y_pos <= HEIGHT_SPACE/sc)
	walls_2[1] = ( x_pos >= WIDTH_SPACE/sc - tl and x_pos <= WIDTH_SPACE/sc ) and ( y_pos >= tl and y_pos <= HEIGHT_SPACE/sc - tl)
	walls_2[2] =  ( x_pos >= 2750/sc + tl and x_pos <= WIDTH_SPACE/sc ) and ( y_pos >= 0 and y_pos <= tl )
	in_obstacle_5 = any(walls_2)
	if in_obstacle_5:
		#print(f'walls right detected')
		return True
	return False

#TODO: MAZE COMPETITION
def check_in_obstacle_two(state, constraints):
	return True

