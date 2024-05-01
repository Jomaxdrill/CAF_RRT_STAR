import numpy as np
import time
import math
import copy

#*robot specifications for turtlebot waffle
RADIUS_ROBOT = 220 #mm
RADIUS_WHEELS = 33 #mm
WHEEL_DISTANCE = 287 #mm
#MAX_ROT_SPEED = 1.82 / 2 # rad/s
MAX_LIN_SPEED = (0.26 / 2) * 1000 # m/s #mm/s

#*robot action parameter
DUR_ACTION = 0.20#0.25 #seconds
STEP_SIZE = MAX_LIN_SPEED * DUR_ACTION

#*algoritm parameters
NUMBER_NODES = 10000 #limit of nodes to create
NEIGHBOR_RADIUS = 25 #mm define end condition for algorithm, trees meet
DEPTH_ANCESTRY = 2 # specify how many nodes before to search for a parent
INTER_POINTS = 30#20 # for check collision, discretization of the corresponding action
DIST_CONNECT_TREES = 25# mm , check for nearby trees

#*default parameters
DEF_BORDER = 10
DEF_WIDTH = 6000
DEF_HEIGHT = 2000

#TODO: REFACTOR this function to be just checking a matrix generated
def check_in_obstacle(state, constraints):
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
	tl = (BORDER + RADIUS_ROBOT) / sc
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

def interpolate_line(start_point, end_point):
	points = []
	for idx in range(INTER_POINTS):
		t_idx = idx / (INTER_POINTS - 1)  # Calculate the parameter t
		x_point = start_point[0] + t_idx * (end_point[0] - start_point[0])  # Linear interpolation for x coordinate
		y_point = start_point[1] + t_idx * (end_point[1] - start_point[1])  # Linear interpolation for y coordinate
		points.append((x_point, y_point))
	return points

def collision(node, new_node, constraints):
	hit = False
	line = interpolate_line(node, new_node)
	for point in line:
		if check_in_obstacle(point, constraints):
			hit = True
			break
	return hit

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
	Return a point along the line segment AB with a fixed step size.

	A, B: Tuple of (x, y) coordinates for points A and B.
	step_size: Fixed step size along the line segment.
	"""
	# Normalize a direction vector from A to B
	norm_dir_vect = normalize_vector(state, sample)
	# Calculate the displacement vector with fixed step size
	displacement_vector = (norm_dir_vect[0] * STEP_SIZE, norm_dir_vect[1] * STEP_SIZE)
	# Calculate the point along the line segment with the fixed step size
	step_applied = (state[0] + displacement_vector[0], state[1] + displacement_vector[1])
	return (round(step_applied[0]),round(step_applied[1]))


def sample_point(constraints):
	_, WIDTH_SPACE, HEIGHT_SPACE = constraints
	# Placeholder sampling function
	rand_width = np.random.uniform(0,1, None)
	rand_height = np.random.uniform(0,1, None)
	random_point = (int(WIDTH_SPACE* rand_width) , int(HEIGHT_SPACE* rand_height))
	return random_point

def nearest(tree, sample):
	min_distance = np.inf
	nearest_node = None

	for node in tree:
		dist = distance(node, sample)
		if dist < min_distance:
			min_distance = dist
			nearest_node = node
	return nearest_node

def steer(nearest_node, sample, tree_A, tree_B, constraints):
	#there is a possibility that nearest node and sample are the same node
	if nearest_node == sample:
		return None
	node_action = advance_straight(nearest_node, sample)
	hit = collision(nearest_node, node_action, constraints)
	#node must be unique for both trees
	if node_action in tree_A or node_action in tree_B:
		#print('already in tree A or B')
		hit = True
	return None if hit else node_action

def find_near_nodes(new_node, tree):
	return set([node for node in tree if distance(new_node, node) < NEIGHBOR_RADIUS])

def ancestry(tree, near_nodes):
	ancestors = set()
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

def choose_parent(neighborhood, new_node, nearest, tree, constraints):
	final_parent = nearest
	nearest_cost = tree[nearest]['cost']
	cost_to_new_nearest = distance(nearest, new_node)
	cost_min = nearest_cost + cost_to_new_nearest
	for node in neighborhood:
		cost_node = tree[node]['cost']
		cost_to_new = distance(node, new_node)
		if (cost_node + cost_to_new) < cost_min:
			if not collision(node, new_node, constraints):
				cost_min = cost_node + cost_to_new
				final_parent = node
	return final_parent, cost_min

def rewire(tree, new_node, near_nodes, history_nodes, constraints):
	for node in near_nodes:
		#checked if a path through either xnew or the parent of xnew makes a path with less cost
		for node_from in [new_node, tree[new_node]['parent']]:
			cost_from_to_near = tree[node_from]['cost'] + distance(node_from, node)
			cost_node = tree[node]['cost']
			if cost_from_to_near < cost_node:
				#check the line from this node to near node does not collide with obstacles
				if not collision(node_from, node, constraints):
					#print(f'rewired node {node} to {node_from}')
					tree[node]['parent'] = node_from
					tree[node]['cost'] = cost_from_to_near
					history_nodes[node]['parent'].append(node_from)

def caf_rrt_star(initial_state, goal_state, constraints):
	"""Creates the State space of all possible movements until goal state is reached by applying the CAF-RRT* algorithm.

	Args:
			initial_state (tuple): multi dimensional tuple 3x3 that describes the initial configuarion of the puzzle
			goal_state (tuple): multi dimensional tuple 3x3 that describes the final configuration the algorithm must find.

	Returns:
			str: 'DONE'. The process have ended thus we have a solution in the tree structure generated.
			str: 'No solution'. The process ended without finding a solution all the tree was traversed.
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
		new_node = steer(nearest_node, random_sample, search_tree_A, search_tree_B, constraints)
		if new_node:
			counter += 1
			#*QUICK RRT* APPROACH*
			order_nodes.append(new_node)
			rand_samples.append(random_sample)
			history_nodes[new_node] = {'parent': []}
			near_nodes = find_near_nodes(new_node, search_tree_A)
			ancestors = ancestry(search_tree_A, near_nodes)
			new_parent, new_cost = choose_parent(near_nodes | ancestors, new_node, nearest_node, search_tree_A, constraints)
			history_nodes[new_node]['parent'].append(new_parent)
			#connect node to Tree
			search_tree_A[new_node] = { 'parent': new_parent, 'cost': new_cost }
			rewire(search_tree_A, new_node, near_nodes, history_nodes, constraints)
			#*bidirectional RRT approach
			nearest_node_B = nearest(search_tree_B, new_node)
			if not collision(new_node, nearest_node_B, constraints) and distance(new_node, nearest_node_B) < DIST_CONNECT_TREES:
				end_time = time.time()
				print( f'DONE in {end_time-start_time} seconds and {counter} nodes were generated.' )
				return {'tree_A': search_tree_A,
						'tree_B': search_tree_B,
						'connector_tree_A': new_node,
						'connector_tree_B': nearest_node_B,
						'record': { 'rand_samples': rand_samples,
									'history_nodes': history_nodes,
									'order_nodes': order_nodes,
									'duration': end_time-start_time,
									}
						}
			else:
				#swap trees
				# copy_search_A = copy.deepcopy(search_tree_A)
				# search_tree_A = copy.deepcopy(search_tree_B)
				# search_tree_B = copy_search_A
				search_tree_A, search_tree_B = search_tree_B, search_tree_A
	end_time = time.time()
	print(f'No solution found. Process took {end_time-start_time} seconds.')
	return {'tree_A': search_tree_A,
			'tree_B': search_tree_B,
			'record': { 'rand_samples': rand_samples,
						'history_nodes': history_nodes,
						'order_nodes': order_nodes,
						'duration': end_time-start_time,
						}
			}

def plan_caf_rrt_star(init, goal, constraints = (DEF_BORDER, DEF_WIDTH, DEF_HEIGHT)):
	initial_hit = check_in_obstacle(init[0:2], constraints)
	goal_hit = check_in_obstacle(goal[0:2], constraints)
	#verify positions are valid
	hit = initial_hit or goal_hit
	if hit:
		print("One or both coordinates hit obstacle space. Please run the program again.")
		return None
	return caf_rrt_star(init[0:2], goal[0:2], constraints)

if __name__ == "__main__":
	plan_caf_rrt_star()

