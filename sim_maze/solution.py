import numpy as np
import math
import time
import matplotlib.pyplot as plt
from maze import distance, collision
DELTA_ERROR = 12.5 #5 if caf 0.1 distance, 10 if caf 0.25-0.75 never more than 15 by now
PROPORTION = 0.1 #BETWEEN 0.01 and 0.1
SMOOTH_FACTOR = 5 #2 no more than 10 for now
INTER_POINTS = 30
INTER_CIRCLE = 10
def backtracking(tree, node):
	"""BackTracking from given node to initial_node
	Args:
		node (Node): Current node to evaluate its parent (previous move done).
	Returns:
		Boolean: True if no more of the path are available
	"""
	path_to_origin = []
	while node is not None:
		path_to_origin.append(node)
		parent = tree[node]['parent']
		node = parent
	return path_to_origin

def initial_path(**result):
	"""
	Generates the initial path by backtracking from the goal node to the start node in both trees.
	The path is then combined and checked for correct order. The total cost of the path is also calculated.

	Parameters:
	result (dict): A dictionary containing the following keys:
		- 'tree_A': The tree from which to start backtracking from the goal node.
		- 'connector_tree_A': The goal node in tree_A.
		- 'tree_B': The tree from which to start backtracking from the goal node.
		- 'connector_tree_B': The goal node in tree_B.
		- 'to': The start node.

	Returns:
	list: The initial path from start to goal, with the first element being the start node.
	"""
	path_A = backtracking(result['tree_A'], result['connector_tree_A'])[::-1]
	path_B = backtracking(result['tree_B'], result['connector_tree_B'])
	cost_A = result['tree_A'][result['connector_tree_A']]['cost']
	cost_B = result['tree_B'][result['connector_tree_B']]['cost']
	cost_connection = distance(result['connector_tree_A'],result['connector_tree_B'])
	cost_total = cost_A + cost_connection + cost_B
	# print(path_A[0],path_B[0], path_B[-1])
	path_total = path_A + path_B
	#guarantee correct order of initial path, first element sohuld be goal
	if path_total[0] != result['to']:
		path_total = path_total[::-1]
	print(f'solution has cost of {cost_total} mm')
	return path_total

def angle(node_a, node_b):
	"""
	Calculates the angle between two points in a 2D plane.

	Parameters:
	node_a (tuple): A tuple representing the (x, y) coordinates of the first point.
	node_b (tuple): A tuple representing the (x, y) coordinates of the second point.

	Returns:
	float: The angle in radians between the two points, measured counterclockwise from the positive x-axis.
	"""
	return math.atan2(node_b[1] - node_a[1], node_b[0] - node_a[0])

def delete_and_insert_by_pos(node_array, index, new_points):
	"""
	This function deletes elements from a list at a specific index and inserts new elements at that index.

	Parameters:
	node_array (list): The original list from which elements will be deleted and new elements will be inserted.
	index (int): The index at which the deletion and insertion operations will be performed.
	new_points (list): The list of new elements to be inserted at the specified index.

	Returns:
	list: A new list with the specified elements deleted and the new elements inserted at the specified index.

	Example:
	>>> node_array = [1, 2, 3, 4, 5]
	>>> index = 2
	>>> new_points = [6, 7]
	>>> delete_and_insert_by_pos(node_array, index, new_points)
	[1, 2, 6, 7, 4, 5]
	"""
	return node_array[:index] + new_points + node_array[index+1:]

def optimize_by(criteria, node_array, use_len, option, matrix, constraints):
	"""
	This function optimizes a given path by equalizing distances or proportions between consecutive points.

	Parameters:
	criteria (float): The criteria for optimization, either a distance or a proportion.
	node_array (list): The original path to be optimized.
	use_len (bool): A flag indicating whether to use the distance between consecutive points for optimization.
	option (str): The option for collision checking.
	matrix (np.ndarray): The matrix for collision checking.
	constraints (dict): The constraints for collision checking.

	Returns:
	list: The optimized path.

	The function iterates through the given path, calculates the distances or proportions between consecutive points,
	and generates new points based on the criteria. If the new points do not result in a collision, they are added to the optimized path.
	"""
	node_array_opt = node_array.copy()
	new_nodes_added = 0
	for idx in range(2, len(node_array) - 1):
		l_1 = distance(node_array[idx], node_array[idx-1]) if use_len else 1
		l_2 = distance(node_array[idx], node_array[idx+1]) if use_len else 1

		theta1 = angle(node_array[idx], node_array[idx-1])
		theta2 = angle(node_array[idx], node_array[idx+1])

		new_prev_node_x = node_array[idx][0] + l_1* criteria * math.cos(theta1)
		new_prev_node_y = node_array[idx][1] + l_1 * criteria * math.sin(theta1)
		new_prev_node = (new_prev_node_x, new_prev_node_y)

		new_aft_node_x = node_array[idx][0] + l_2 * criteria * math.cos(theta2)
		new_aft_node_y = node_array[idx][1] + l_2 * criteria * math.sin(theta2)
		new_aft_node = (new_aft_node_x, new_aft_node_y)
		if not collision(new_prev_node,new_aft_node, option, constraints, INTER_POINTS, matrix):
			node_array_opt = delete_and_insert_by_pos(node_array_opt, idx + new_nodes_added, [new_prev_node, new_aft_node])
			new_nodes_added += 1
	return node_array_opt

def optimize_path(first_path, delta_e, p_val, option, matrix, constraints):
	start_time = time.time()
	path_sol_to_opt = first_path.copy()
	path_by_equal_dist = None
	path_by_equal_prop = None
	# First loop (j from 1 to 2)
	for j_idx in range(1, 3):
		# Optimize by equalization method
		path_by_equal_dist = optimize_by(delta_e, path_sol_to_opt, False, option, matrix, constraints)
		#Optimize by equal proportion method
		path_by_equal_prop = optimize_by(p_val, path_by_equal_dist, True, option, matrix, constraints)
	# Check collisions
	index_to_delete = []
	for idx in range(2, len(path_by_equal_prop) - 1):
		if collision(path_by_equal_prop[idx-1], path_by_equal_prop[idx+1], option, constraints, INTER_POINTS, matrix):
			index_to_delete.append(idx)
	final_opt_path = [ elem for idx, elem in enumerate(path_by_equal_prop) if idx not in index_to_delete ]
	#calculate cost optimal path
	cost_opt = 0
	for idx in range(len(final_opt_path)-1):
		cost_opt += distance(final_opt_path[idx], final_opt_path[idx+1])
	end_time = time.time()
	print(f'Optimal solution strategy took {end_time - start_time} seconds and has cost of {cost_opt} mm')
	return final_opt_path, cost_opt


def interpolate_points_on_arc(center, radius, beta1, beta2):
	"""
	This function calculates and returns a list of points that lie on an arc.

	Parameters:
	center (tuple): A tuple representing the (x, y) coordinates of the center of the arc.
	radius (float): The radius of the arc.
	beta1 (float): The angle in radians representing the starting point of the arc.
	beta2 (float): The angle in radians representing the ending point of the arc.

	Returns:
	list: A list of tuples, where each tuple represents the (x, y) coordinates of a point on the arc.

	The function calculates the interpolated points on the arc by iterating over a range of angles
	from beta1 to beta2. For each angle, it calculates the corresponding (x, y) coordinates using
	the formula for points on a circle and appends them to the list of interpolated points.
	"""
	interpolated_points = []
	for idx in range(INTER_CIRCLE):
		# Calculate the angle for this point along the arc
		beta_interpolated = beta1 + (beta2 - beta1) * idx / (INTER_CIRCLE - 1)
		# Calculate the coordinates of the interpolated point
		x_interpolated = center[0] + radius * math.cos(beta_interpolated)
		y_interpolated = center[1] + radius * math.sin(beta_interpolated)
		interpolated_points.append((x_interpolated, y_interpolated))
	return interpolated_points



def smooth_path(optimal_path, cost_optimal, smooth_factor, constraints = None):
	"""
	This function smooths a given path by creating smooth arcs between consecutive points.

	Parameters:
	optimal_path (list): The original path to be smoothed.
	cost_optimal (float): The cost of the original path.
	smooth_factor (float): The factor by which to reduce the length of the arcs.
	constraints (dict, optional): The constraints for collision checking. Defaults to None.

	Returns:
	tuple: A tuple containing the smoothed path and the cost of the smoothed path.

	The function iterates through the given path, calculates the angles and distances between consecutive points,
	and generates new points along the arcs between the original points. If the new points do not result in a collision,
	they are added to the smoothed path. The cost of the smoothed path is calculated using the distance function.
	"""
	start_time = time.time()
	final_smoothed_path = optimal_path.copy()
	new_nodes_added = 0
	cost_smooth = 0
	for idx in range(2, len(optimal_path) - 1):

		l_1 = distance(optimal_path[idx], optimal_path[idx-1])
		l_2 = distance(optimal_path[idx], optimal_path[idx+1])

		beta1 = angle(optimal_path[idx], optimal_path[idx-1])
		beta2 = angle(optimal_path[idx], optimal_path[idx+1])

		l_min = min(l_1, l_2) /smooth_factor

		arc_init_x = optimal_path[idx][0] + l_min * math.cos(beta1)
		arc_init_y = optimal_path[idx][1] + l_min * math.sin(beta1)

		arc_end_x = optimal_path[idx][0] + l_min * math.cos(beta2)
		arc_end_y = optimal_path[idx][1] + l_min * math.sin(beta2)

		arc_init = (arc_init_x,arc_init_y)
		arc_end = (arc_end_x,arc_end_y)
		#? FINDS A OPTIMAL RADIUS AND CENTER OF THE CUTTING CIRCLE WHO CREATES THE SMOOTH ARC
		#TODO: Improve calculation of arc length and angle identities usage, giving anomal results
		#*So currently the function applies as paper explains too the use of equal proportion strategy
		#* To create sub smooth arcs which only need the start and end of the arc as the new points to be added
		# angle_diff = beta1 - beta2
		# if angle_diff > math.pi:
		# 	angle_diff -= 2 * math.pi
		# elif angle_diff < -math.pi:
		# 	angle_diff += 2 * math.pi
		# radius = l_min * math.tan((angle_diff) / 2)

		# circle_arc_x = arc_init_x + radius * math.cos(beta1 + math.pi / 2)
		# circle_arc_y = arc_init_y + radius * math.sin(beta1 + math.pi / 2)

		# circle_arc_x_1 = arc_init_x + radius * math.cos(beta1 - math.pi / 2)
		# circle_arc_y_1 = arc_init_y + radius * math.sin(beta1 - math.pi / 2)

		# circle_arc_x_2 = arc_init_x + radius * math.cos(beta2 + math.pi / 2)
		# circle_arc_y_2 = arc_init_y + radius * math.sin(beta2 + math.pi / 2)

		# circle_arc_x_3 = arc_init_x + radius * math.cos(beta2 - math.pi / 3)
		# circle_arc_y_3 = arc_init_y + radius * math.sin(beta2 - math.pi / 3)

		# if (circle_arc_x_1 == circle_arc_x_2 and
		# 	circle_arc_y_1 == circle_arc_y_2) or (circle_arc_x_1 == circle_arc_x_3 and circle_arc_y_1 == circle_arc_y_3):
		# 	circle_arc_x = circle_arc_x_1
		# 	circle_arc_y = circle_arc_y_1
		# optimal_center = (circle_arc_x, circle_arc_y)
		# #TODO: Change calculation cost to a better computation as the one in the paper
		# #arc_smooth = arc_length_radians(angle(arc_init, arc_end), radius)
		#assuming cost_smooth starts as equal to cost_optimal
		# #cost_smooth = cost_smooth - (l_1 + l_2) + arc_smooth + distance(optimal_path[idx-1],arc_init) + distance(arc_end,optimal_path[idx+1])
		points_arc = [] #interpolate_points_on_arc(optimal_center, radius, beta1, beta2)
		# #? END FIND AN OPTIMAL CENTER AND RADIUS
		#modify the path by adding the arc
		arc_to_add = [arc_init] + points_arc + [arc_end]
		final_smoothed_path = delete_and_insert_by_pos(final_smoothed_path, idx + new_nodes_added, arc_to_add)
		new_nodes_added += 1
	end_time = time.time()

	#*The way suggested by paper calculates considers a continuos domain, as here interpolation has been applied
	#* A good aproximation for cost can be computed as before using distance function
	for idx in range(len(final_smoothed_path)-1):
		cost_smooth += distance(final_smoothed_path[idx], final_smoothed_path[idx+1])
	print(f'Smooth the optimal took {end_time - start_time} seconds and has cost of {cost_smooth} mm')
	return final_smoothed_path, cost_smooth

def to_robot_coordinate(path):
	"""
	Converts a path from world coordinates to robot coordinates.

	Parameters:
	path (list): A list of tuples representing the (x, y) coordinates in world coordinates.

	Returns:
	list: A list of tuples representing the (x, y) coordinates in robot coordinates.

	The function calculates the transformation matrix to convert from world coordinates to robot coordinates.
	The transformation matrix is defined by the initial point of the path in world coordinates.
	The function then iterates through the given path, applies the transformation matrix to each point,
	and appends the transformed point to the robot_path list.

	Note:
	The transformation matrix is defined as follows:
	[ [1,0, -x_init],[0, 1, -y_init],[0, 0, 1] ]
	where x_init and y_init are the coordinates of the initial point in world coordinates.

	Example:
	>>> path = [(100, 200), (200, 300), (300, 400)]
	>>> to_robot_coordinate(path)
	[(-100, -200), (0, 0), (100, 100)]
	"""
	#get the point that states the coordinates of the robot, in robot coordinates is the 0,0 vector
	x_init, y_init = path[0] # in mm
	#define the transf from world coord to robot coordinate
	TRANS_MATRIX = [ [1,0, -x_init],[0, 1, -y_init],[0, 0, 1] ]
	robot_path = []
	for coord in path:
		x_robot, y_robot, _ = np.dot(TRANS_MATRIX, (coord[0], coord[1], 1))
		robot_path.append((x_robot, y_robot))
	return robot_path

#*OPTIONAL********************************
def plot_paths(paths_sol):
	"""
	This function plots multiple paths on a 2D graph for visualization.

	Parameters:
	paths_sol (list): A list of tuples, where each tuple represents a path. Each path is a list of (x, y) coordinates.

	Returns:
	None

	The function extracts the x and y coordinates from each path and plots them on a graph using matplotlib.
	The paths are labeled with their respective names ('Initial Path', 'Optimal Path', 'Smoothed Path') for clarity.
	The graph is then configured with a title, axis labels, legend, and grid for better visualization.
	Finally, the graph is displayed using plt.show().
	"""
	names = ['Initial Path', 'Optimal Path', 'Smoothed Path']
	for idx, coords in enumerate(paths_sol):
		x_elements = [x for x, _ in coords]
		y_elements = [y for _, y in coords]
		plt.plot(x_elements, y_elements, label = f'{names[idx]}')
	plt.title("CAF-RRT* paths")
	plt.xlabel("X position [mm]")
	plt.ylabel("Y position [mm]")
	plt.legend()
	plt.grid()
	plt.show()

def generate_path(constraints, option, matrix, **result):
	"""
	Generates a path from start to goal using the CAF-RRT* algorithm.

	Parameters:
	constraints (dict): A dictionary containing the constraints for collision checking.
	option (str): The option for collision checking.
	matrix (np.ndarray): The matrix for collision checking.
	result (dict): A dictionary containing the following keys:
		- 'tree_A': The tree from which to start backtracking from the goal node.
		- 'connector_tree_A': The goal node in tree_A.
		- 'tree_B': The tree from which to start backtracking from the goal node.
		- 'connector_tree_B': The goal node in tree_B.
		- 'to': The start node.

	Returns:
	tuple: A tuple containing the initial path, the optimized path, the smoothed path, and the robot path.

	The function first generates the initial path using the initial_path function.
	Then, it optimizes the initial path using the optimize_path function.
	Next, it smooths the optimized path using the smooth_path function.
	Finally, it converts the smoothed path to robot coordinates using the to_robot_coordinate function.
	"""
	init_path = initial_path(**result)
	opt_path, cost_opt = optimize_path(init_path, DELTA_ERROR, PROPORTION, option, matrix, constraints)
	smth_path, cost_smooth = smooth_path(opt_path, cost_opt, SMOOTH_FACTOR)
	robot_path = to_robot_coordinate(smth_path[::-1]) #from start
	#plot_paths((init_path,opt_path,smth_path))
	return init_path, opt_path, smth_path, robot_path
