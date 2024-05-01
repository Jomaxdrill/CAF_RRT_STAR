import numpy as np
import math
import time
from caf_rrt_star import distance, collision
DELTA_ERROR = 10 #5 if caf 0.1 distance, never more than 15 by now
PROPORTION = 0.03
SMOOTH_FACTOR = 20#2
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
	path_A = backtracking(result['tree_A'], result['connector_tree_A'])[::-1]
	path_B = backtracking(result['tree_B'], result['connector_tree_B'])
	cost_A = result['tree_A'][result['connector_tree_A']]['cost']
	cost_B = result['tree_B'][result['connector_tree_B']]['cost']
	cost_connection = distance(result['connector_tree_A'],result['connector_tree_B'])
	cost_total = cost_A + cost_connection + cost_B
	path_total = path_A + path_B
	print(f'solution has cost of {cost_total} mm')
	return path_total
#TODO: GAURAV
def angle(node_a, node_b):
	return math.atan2(node_b[1] - node_a[1], node_b[0] - node_a[0])

def delete_by_pos(node_array, index):
	# Delete logic for removing vertex from data structure
	"""Remove element at the specified index from the array."""
	# if index < 0 or index >= len(node_array):
	#     raise IndexError("Index out of range")
	return node_array[:index] + node_array[index+1:]

def delete_and_insert_by_pos(node_array, index, new_points):
	return node_array[:index] + new_points + node_array[index+1:]

def optimize_by(criteria, node_array, use_len, constraints, collision):
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
		if not collision(new_prev_node,new_aft_node, constraints):
			node_array_opt = delete_and_insert_by_pos(node_array_opt, idx + new_nodes_added, [new_prev_node, new_aft_node])
			new_nodes_added += 1
	return node_array_opt

def optimize_path(first_path, delta_e, p_val, collision , constraints):
	start_time = time.time()
	path_sol_to_opt = first_path.copy()
	path_by_equal_dist = None
	path_by_equal_prop = None
	# First loop (j from 1 to 2)
	for j_idx in range(1, 3):
		# Optimize by equalization method
		path_by_equal_dist = optimize_by(delta_e, path_sol_to_opt, False, constraints, collision)
		#Optimize by equal proportion method
		path_by_equal_prop = optimize_by(p_val, path_by_equal_dist, True, constraints, collision)
	# Fourth loop (k from 2 to Nopt - 1)
	index_to_delete = []
	for idx in range(2, len(path_by_equal_prop) - 1):
		if collision(path_by_equal_prop[idx-1], path_by_equal_prop[idx+1], constraints):
			index_to_delete.append(idx)
	final_opt_path = [ elem for idx, elem in enumerate(path_by_equal_prop) if idx not in index_to_delete ]
	#calculate cost optimal path
	cost_opt = 0
	for idx in range(len(final_opt_path)-1):
		cost_opt += distance(final_opt_path[idx], final_opt_path[idx+1])
	end_time = time.time()
	print(f'Optimal solution strategy took {end_time - start_time} seconds and has cost of {cost_opt} mm')
	return final_opt_path, cost_opt

#TODO: NAGA
def smooth_path(optimal_path, smooth_factor, constraints):
	start_time = time.time()
	final_smoothed_path = optimal_path.copy()
	optimal_radius = {}
	new_nodes_added = 0

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
		#? FIND A OPTIMAL RADIUS AND OPTIMAL CIRCLE WHERE THE ARC IS PART OF
		radius = abs(l_min * math.tan((beta1 - beta2) / 2))

		circle_arc_x = arc_init_x + radius * math.cos(beta1 + math.pi / 2)
		circle_arc_y = arc_init_y + radius * math.sin(beta1 + math.pi / 2)

		circle_arc_x_1 = arc_init_x + radius * math.cos(beta1 - math.pi / 2)
		circle_arc_y_1 = arc_init_y + radius * math.sin(beta1 - math.pi / 2)

		circle_arc_x_2 = arc_init_x + radius * math.cos(beta2 + math.pi / 2)
		circle_arc_y_2 = arc_init_y + radius * math.sin(beta2 + math.pi / 2)

		circle_arc_x_3 = arc_init_x + radius * math.cos(beta2 - math.pi / 3)
		circle_arc_y_3 = arc_init_y + radius * math.sin(beta2 - math.pi / 3)

		if (circle_arc_x_1 == circle_arc_x_2 and
			circle_arc_y_1 == circle_arc_y_2) or (circle_arc_x_1 == circle_arc_x_3 and circle_arc_y_1 == circle_arc_y_3):
			circle_arc_x = circle_arc_x_1
			circle_arc_y = circle_arc_y_1
		optimal_radius[idx] = (circle_arc_x, circle_arc_y)
		#? END FIND AN OPTIMAL RADIUS
		final_smoothed_path = delete_and_insert_by_pos(final_smoothed_path, idx + new_nodes_added, [arc_init, arc_end])
		new_nodes_added += 1
		# if not collision(arc_init, arc_end, constraints):
		# 	final_smoothed_path = delete_and_insert_by_pos(final_smoothed_path, idx + new_nodes_added, [arc_init, arc_end])
		# 	new_nodes_added += 1
		#TODO: NAGA----- CALCULATE SMOOTH PATH COST
	end_time = time.time()
	#calculate cost optimal path
	cost_smooth = 0
	for idx in range(len(final_smoothed_path)-1):
		cost_smooth += distance(final_smoothed_path[idx], final_smoothed_path[idx+1])
	print(f'Smooth the optimal took {end_time - start_time} seconds and has cost of {cost_smooth} mm')
	return final_smoothed_path, cost_smooth, optimal_radius

def generate_path(constraints,**result):
	init_path = initial_path(**result)
	opt_path, cost_opt = optimize_path(init_path, DELTA_ERROR, PROPORTION, collision , constraints)
	smth_path, cost_smooth, opt_radius = smooth_path(opt_path, SMOOTH_FACTOR, constraints)
	return init_path, opt_path, smth_path

if __name__ == "__main__":
	generate_path()