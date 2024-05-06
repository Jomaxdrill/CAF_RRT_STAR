import numpy as np
import time
import math
from maze import check_in_obstacle, collision, distance, get_vector

#*robot specifications for turtlebot waffle
# RADIUS_ROBOT = 220 #mm
# RADIUS_WHEELS = 33 #mm
# WHEEL_DISTANCE = 287 #mm
#MAX_ROT_SPEED = 1.82 / 2 # rad/s
MAX_LIN_SPEED = (0.26 / 2) * 1000 # m/s #mm/s

#*robot action parameter
DUR_ACTION = 0.1#0.25 #seconds
STEP_SIZE = MAX_LIN_SPEED * DUR_ACTION

#*algoritm parameters
NUMBER_NODES = 10000 #limit of nodes to create
NEIGHBOR_RADIUS = 25 #mm define end condition for algorithm, trees meet
DEPTH_ANCESTRY = 2 # specify how many nodes before to search for a parent
INTER_POINTS = 100##20 # for check collision, discretization of the corresponding action
DIST_CONNECT_TREES = 25# mm , check for nearby trees

#*default parameters
DEF_BORDER = 10
DEF_WIDTH = 6000
DEF_HEIGHT = 2000

def normalize_vector(node_a, node_b):
	"""
	Normalize a 2D vector.
	vector: Tuple of (x, y) components of the vector.
	"""
	x_vect, y_vect = get_vector(node_a, node_b)
	len_vect = distance(node_a, node_b)
	return (x_vect / len_vect, y_vect / len_vect)

def advance_straight(state, sample):
	"""
	This function calculates a new point along the line segment between two given points,
	with a fixed step size. The new point is obtained by normalizing the direction vector
	from the first point to the second point and then multiplying it by the step size.

	Parameters:
	state (tuple): A tuple representing the coordinates of the starting point (x, y).
	sample (tuple): A tuple representing the coordinates of the ending point (x, y).

	Returns:
	tuple: A tuple representing the coordinates of the new point (x, y) obtained by advancing
	straight from the starting point towards the ending point with the fixed step size.
	The coordinates are rounded to the nearest integer.
	"""
	# Normalize a direction vector from A to B
	norm_dir_vect = normalize_vector(state, sample)
	# Calculate the displacement vector with fixed step size
	displacement_vector = (norm_dir_vect[0] * STEP_SIZE, norm_dir_vect[1] * STEP_SIZE)
	# Calculate the point along the line segment with the fixed step size
	step_applied = (state[0] + displacement_vector[0], state[1] + displacement_vector[1])
	return (round(step_applied[0]),round(step_applied[1]))

def sample_point(constraints):
	"""
	This function generates a random point within the given constraints.

	Parameters:
	constraints (tuple): A tuple containing the border width, width of the space, and height of the space.

	Returns:
	tuple: A tuple representing a random point within the given constraints. The coordinates are rounded to the nearest integer.
	"""
	_, WIDTH_SPACE, HEIGHT_SPACE = constraints
	# Placeholder sampling function
	rand_width = np.random.uniform(0,1, None)
	rand_height = np.random.uniform(0,1, None)
	random_point = (int(WIDTH_SPACE* rand_width) , int(HEIGHT_SPACE* rand_height))
	return random_point

def nearest(tree, sample):
	"""
	Finds the nearest node in the given tree to the sample point.

	Parameters:
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.
	sample (tuple): A tuple representing the coordinates of the sample point.

	Returns:
	tuple: The coordinates of the nearest node in the tree to the sample point.

	Note:
	This function iterates over all nodes in the tree to find the node with the minimum distance to the sample point.
	The distance is calculated using the 'distance' function.
	"""
	min_distance = np.inf
	nearest_node = None

	for node in tree:
		dist = distance(node, sample)
		#a neighbor is a node inside this circle of vicinity
		if dist < min_distance:
			min_distance = dist
			nearest_node = node
	return nearest_node

def steer(nearest_node, sample, tree_A, tree_B, option, matrix, constraints):
	"""
	Finds the nearest node in the given tree to the sample point.

	Parameters:
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.
	"""
	#there is a possibility that nearest node and sample are the same node
	if nearest_node == sample:
		return None
	node_action = advance_straight(nearest_node, sample)
	hit = collision(nearest_node, node_action, option, constraints, INTER_POINTS, matrix)
	#node must be unique for both trees
	if node_action in tree_A or node_action in tree_B:
		#print('already in tree A or B')
		hit = True
	return None if hit else node_action

def find_near_nodes(new_node, tree):
	"""
	Finds the nodes in the given tree that are within a specified neighborhood radius of the new node.

	Parameters:
	new_node (tuple): A tuple representing the coordinates of the new node.
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.

	Returns:
	set: A set of tuples representing the coordinates of the nodes in the tree that are within the neighborhood radius of the new node.

	Note:
	This function iterates over all nodes in the tree to find the nodes with a distance less than the specified NEIGHBOR_RADIUS to the new node.
	The distance is calculated using the 'distance' function.
	"""
	return set([node for node in tree if distance(new_node, node) < NEIGHBOR_RADIUS])

def ancestry(tree, near_nodes):
	"""
	Finds the ancestors of the given nodes in the tree up to a specified depth level.
	An ancestor can be considered nodes from previous samplings that connected many nodes after

	Parameters:
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.
	near_nodes (set): A set of tuples representing the coordinates of the nodes in the tree that are within the neighborhood radius of the new node.

	Returns:
	set: A set of tuples representing the coordinates of the ancestors of the given nodes in the tree up to the specified depth level.

	Note:
	This function iterates over the given near_nodes and finds their ancestors in the tree.
	The ancestors are determined by following the parent links of each node up to the specified depth level (DEPTH_ANCESTRY).
	The function returns a set of unique ancestor coordinates.
	"""
	ancestors = set() #ancestors must be unique, two nodes might have the same ancestor
	for node in near_nodes:
		#for each nearby node find ancestors and save them until DEPTH level has been reached
		level = 0
		ancestors.add(node)
		current = node
		while level <= DEPTH_ANCESTRY:
			parent = tree[current]['parent']
			if parent is None:
				break
			ancestors.add(parent)
			current = parent
			level +=1
	return ancestors

def choose_parent(neighborhood, new_node, nearest, tree, option, matrix, constraints):
	"""
	This function selects the best parent for the new node based on cost and collision checks.

	Parameters:
	neighborhood (set): A set of tuples representing the coordinates of the nodes in the tree that are within the neighborhood radius of the new node.
	new_node (tuple): A tuple representing the coordinates of the new node.
	nearest (tuple): A tuple representing the coordinates of the nearest node in the tree to the new node.
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.
	option (str): A string representing the option for the algorithm.
	matrix (numpy.ndarray): A 2D numpy array representing the map or environment.
	constraints (tuple): A tuple containing the border width, width of the space, and height of the space.

	Returns:
	tuple: A tuple containing the coordinates of the selected parent node and the corresponding cost.

	Note:
	This function iterates over the neighborhood nodes and checks if a path through either the new node or the parent of the new node makes a path with less cost.
	It also checks if the line from the selected parent to the new node does not collide with obstacles.
	If a better parent is found, the function updates the selected parent and the corresponding cost.
	"""
	final_parent = nearest
	nearest_cost = tree[nearest]['cost']
	cost_to_new_nearest = distance(nearest, new_node)
	cost_min = nearest_cost + cost_to_new_nearest
	#check between near nodes and ancestors for the parent - one which arrive to new node has the least cost of all
	for node in neighborhood:
		cost_node = tree[node]['cost']
		cost_to_new = distance(node, new_node)
		if (cost_node + cost_to_new) < cost_min:
			if not collision(node, new_node, option, constraints, INTER_POINTS, matrix):
				cost_min = cost_node + cost_to_new
				final_parent = node
	return final_parent, cost_min

def rewire(tree, new_node, near_nodes, history_nodes, option, matrix, constraints):
	"""
	Rewire the tree to improve the path cost by checking if a path through either the new node or the parent of the new node makes a path with less cost.
	Also, check if the line from the selected parent to the new node does not collide with obstacles.

	Parameters:
	tree (dict): A dictionary representing the search tree. The keys are the node coordinates (tuples), and the values are dictionaries containing the 'parent' and 'cost' of each node.
	new_node (tuple): A tuple representing the coordinates of the new node.
	near_nodes (set): A set of tuples representing the coordinates of the nodes in the tree that are within the neighborhood radius of the new node.
	history_nodes (dict): A dictionary to keep track of the history of parent changes for each node.
	option (str): A string representing the option for the algorithm.
	matrix (numpy.ndarray): A 2D numpy array representing the map or environment.
	constraints (tuple): A tuple containing the border width, width of the space, and height of the space.

	Returns:
	None: The function modifies the tree, history_nodes in place.
	"""
	for node in near_nodes:
		#checked if a path through either xnew or the parent of xnew makes a path with less cost
		for node_from in [new_node, tree[new_node]['parent']]:
			cost_from_to_near = tree[node_from]['cost'] + distance(node_from, node)
			cost_node = tree[node]['cost']
			if cost_from_to_near < cost_node:
				#check the line from this node to near node does not collide with obstacles
				if not collision(node_from, node, option, constraints, INTER_POINTS, matrix):
					#print(f'rewired node {node} to {node_from}')
					tree[node]['parent'] = node_from
					tree[node]['cost'] = cost_from_to_near
					history_nodes[node]['parent'].append(node_from)

def caf_rrt_star(initial_state, goal_state, option, matrix, constraints):
	"""
	CAF-RRT* algorithm for path planning in a 2D space.

	Parameters:
	initial_state (tuple): The initial state (x, y) coordinates of the robot.
	goal_state (tuple): The goal state (x, y) coordinates of the robot.
	option (str): The option for the algorithm, e.g., '2D' or '3D'.
	matrix (numpy.ndarray): A 2D numpy array representing the map or environment.
	constraints (tuple): A tuple containing the border width, width of the space, and height of the space.

	Returns:
	dict: A dictionary containing the search trees, connector nodes, and other relevant information.
	"""
	counter = 0
	start_time = time.time()
	search_tree_A = {}
	search_tree_B = {}
	search_tree_A[initial_state] = { 'parent': None, 'cost': 0 }
	search_tree_B[goal_state] = { 'parent': None, 'cost': 0 }
	#history_track
	rand_samples = [] #for animation purposes
	#for animation purposes, save for each node created how their parents changed
	# a way to show the rewiring process
	history_nodes = {}
	order_nodes = [] #for animation purposes, save the order they where created
	while (counter < NUMBER_NODES):
		# print(counter, end="\r")
		random_sample =  sample_point(constraints)
		nearest_node = nearest(search_tree_A, random_sample)
		new_node = steer(nearest_node, random_sample, search_tree_A, search_tree_B, option, matrix, constraints)
		if new_node:
			counter += 1
			print(counter, end="\r")
			#*QUICK RRT* APPROACH*
			order_nodes.append(new_node)
			rand_samples.append(random_sample)
			history_nodes[new_node] = {'parent': []}
			near_nodes = find_near_nodes(new_node, search_tree_A)
			ancestors = ancestry(search_tree_A, near_nodes)
			new_parent, new_cost = choose_parent(near_nodes | ancestors, new_node, nearest_node, search_tree_A, option, matrix, constraints)
			history_nodes[new_node]['parent'].append(new_parent)
			#connect node to Tree
			search_tree_A[new_node] = { 'parent': new_parent, 'cost': new_cost }
			rewire(search_tree_A, new_node, near_nodes, history_nodes,option, matrix, constraints)
			#*bidirectional RRT approach
			nearest_node_B = nearest(search_tree_B, new_node)
			if not collision(new_node, nearest_node_B, option, constraints, INTER_POINTS, matrix) and distance(new_node, nearest_node_B) < DIST_CONNECT_TREES:
				end_time = time.time()
				print( f'DONE in {end_time-start_time} seconds and {counter} nodes were generated.' )
				return {'tree_A': search_tree_A,
						'tree_B': search_tree_B,
						'connector_tree_A': new_node,
						'connector_tree_B': nearest_node_B,
						'from': initial_state,
						'to': goal_state,
						'record': { 'rand_samples': rand_samples,
									'history_nodes': history_nodes,
									'order_nodes': order_nodes,
									'duration': end_time-start_time,
									}
						}
			else:
				search_tree_A, search_tree_B = search_tree_B, search_tree_A
	end_time = time.time()
	print(f'No solution found. Process took {end_time-start_time} seconds.')
	return {'tree_A': search_tree_A,
			'tree_B': search_tree_B,
			'from': initial_state,
			'to': goal_state,
			'record': { 'rand_samples': rand_samples,
						'history_nodes': history_nodes,
						'order_nodes': order_nodes,
						'duration': end_time-start_time,
						}
			}

def plan_caf_rrt_star(init, goal, option, matrix = None, constraints = (DEF_BORDER, DEF_WIDTH, DEF_HEIGHT)):
	"""
	This function is the entry point for the CAF-RRT* algorithm. It checks if the initial and goal positions are valid,
	and if so, it calls the caf_rrt_star function to perform the path planning.

	Parameters:
	init (tuple): A tuple representing the initial state (x, y) coordinates of the robot.
	goal (tuple): A tuple representing the goal state (x, y) coordinates of the robot.
	option (str): The option for the algorithm, e.g., '2D' or '3D'.
	matrix (numpy.ndarray, optional): A 2D numpy array representing the map or environment. Default is None.
	constraints (tuple, optional): A tuple containing the border width, width of the space, and height of the space. Default is (DEF_BORDER, DEF_WIDTH, DEF_HEIGHT).

	Returns:
	dict: If a valid path is found, the function returns a dictionary containing the search trees, connector nodes, and other relevant information.
		If no valid path is found, the function returns None.
	"""
	initial_hit = check_in_obstacle(init[0:2], option, constraints, matrix)
	goal_hit = check_in_obstacle(goal[0:2], option, constraints, matrix)
	#verify validity of positions
	hit = initial_hit or goal_hit
	if hit:
		print("One or both coordinates hit obstacle space. Please run the program again.")
		return None
	return caf_rrt_star(init[0:2], goal[0:2],option, matrix, constraints)


