import numpy as np
import cv2
from tqdm import tqdm
from maze import coordinate_image

#*animation
COLOR_OBSTACLE = (255, 0, 0)#BLUE
COLOR_BORDER = (255, 255, 255)#WHITE
COLOR_INITIAL = (128, 0, 128) #PURPLE
COLOR_OPTIMAL = (128, 128 ,128)#GRAY
COLOR_SMOOTH = (255, 0, 255) #PINK
COLOR_RADIUS_GOAL = (128, 128 ,0) #TEAL
COLOR_TREE_A =  (0, 255, 255)#YELLOW
COLOR_TREE_B = (0, 0, 255)#RED
COLOR_INIT = (0,125,255)#ORANGE
COLOR_GOAL = (125, 0 ,255) #RASPBERRY
COLOR_REWIRE =  (255,125,0) #OCEAN
COLOR_FINAL_PARENT = (255, 255, 0)#LIGHT BLUE
COLOR_RAND_POINT = (0, 255, 0)#GREEN
FPS = 25
SCALE_FACTOR = 0.5
MAX_FRAME_NODES = 250 #*when frames surpass 300 it can fail the rendering
MAX_GOAL_FRAMES = 90 #*set also limit for frames of goal
NODES_PER_FRAME = 10
LINES_PER_FRAME = 20

#*default parameters
DEF_BORDER = 10
DEF_WIDTH = 6000
DEF_HEIGHT = 2000


def divide_array(vect_per_frame, arr_nodes):
	"""
	This function is used to divide an array into chunks of a specified size.

	Args:
		vect_per_frame (int): The number of nodes to include in each chunk.
		arr_nodes (list): A list of nodes to divide.

	Returns:
		list: A list of lists, where each sub-list represents a chunk of nodes.

	"""
	arr_size = len(arr_nodes)
	if arr_size <= vect_per_frame:
			return [ arr_nodes ]
	# Calculate the number of full chunks and the size of the remaining chunk
	number_full_slices  = arr_size // vect_per_frame
	remaining_slice = arr_size % vect_per_frame
	# Slice the array into chunks of the nodes per frame
	sliced_chunks = [ arr_nodes[idx*vect_per_frame:(idx+1)*vect_per_frame]
				for idx in range(number_full_slices) ]
	# Remaining nodes into a separate chunk
	if remaining_slice > 0:
		sliced_chunks.append(arr_nodes[number_full_slices*vect_per_frame:])
	return sliced_chunks

def create_maze(init, goal, given_map, option, trans_matrix, constraints):
	"""
	Creates a blank image with the outer boundary of the arena drawn on it.
	Draws filled rectangles for the initial and goal states, and outlines for them.
	Defines the polygon points for the rotated hexagon and the polygon.
	Draws the rotated hexagon and the filled polygon, and outlines them.
	Returns:
		np.ndarray: The blank image with the arena drawn on it.
	"""
	print("Generating map...")
	# Get the width and height of the arena
	BORDER, WIDTH_SPACE, HEIGHT_SPACE = constraints
	canvas = None
	if given_map is not None:
		canvas = given_map
	else:
		# Create a blank image
		canvas = np.zeros((HEIGHT_SPACE, WIDTH_SPACE, 3), dtype="uint8")
	#draw initial and goal state points and  radius goal
	y_init, x_init = coordinate_image(init[0:2], trans_matrix)
	y_goal, x_goal = coordinate_image(goal[0:2], trans_matrix)
	draw_circle(canvas, (x_init,y_init) , 15, COLOR_INIT, 1)
	draw_circle(canvas, (x_goal,y_goal) , 15, COLOR_GOAL, 1)
	if option ==1:
		# Draw the outer boundary
		cv2.rectangle(canvas, (0, 0), (WIDTH_SPACE, HEIGHT_SPACE),COLOR_BORDER, int(BORDER)*2)

		# Upper Rectangle
		draw_rectangle(canvas, (1500, 0), (1750, 1000), COLOR_OBSTACLE, int(BORDER))

		# Lower Rectangle
		draw_rectangle(canvas, (2500, 2000), (2750, 1000), COLOR_OBSTACLE, int(BORDER))

		# Circle
		draw_circle(canvas, (4200, 800), 600, COLOR_OBSTACLE, int(BORDER))
	if option ==2:
		pass
	return canvas

def draw_rectangle(canvas, pt1, pt2, color, border):
	"""
	Draws a filled rectangle and its outline on the given canvas.

	Parameters:
	canvas (np.ndarray): The image or canvas on which to draw the rectangle.
	pt1 (tuple): A tuple representing the (x, y) coordinates of the top-left corner of the rectangle.
	pt2 (tuple): A tuple representing the (x, y) coordinates of the bottom-right corner of the rectangle.
	color (tuple): A tuple representing the RGB color of the rectangle.
	border (int): The thickness of the rectangle's border.

	Returns:
	None. The function modifies the canvas in-place.
	"""
	# Draw filled rectangle
	cv2.rectangle(canvas, pt1, pt2, color, -1)
	# Draw outline
	cv2.rectangle(canvas, pt1, pt2, COLOR_BORDER, border)

def draw_circle(canvas, center, radius, color, border):
	"""
	Draws a circle on the given canvas with the specified center, radius, color, and border.

	Parameters:
	canvas (np.ndarray): The image or canvas on which to draw the circle.
	center (tuple): A tuple representing the (x, y) coordinates of the center of the circle.
	radius (int): The radius of the circle.
	color (tuple): A tuple representing the RGB color of the circle.
	border (int): The thickness of the circle's border.

	Returns:
	None. The function modifies the canvas in-place.
	"""
	# Draw filled circle
	cv2.circle(canvas, center, radius, color, -1)
	# Draw outline
	cv2.circle(canvas, center, radius, COLOR_BORDER, border)

def draw_trees(maze, nodes_per_frame, rand_point_per_frame, trans_matrix, result):
	"""
	Draws the trees (RRT and RRT*) in the animation. Each tree is represented as a set of connected lines.
	Random points are also drawn for each frame.

	Args:
		maze (np.ndarray): The image of the maze or arena.
		nodes_per_frame (list): A list of sets, where each set contains the nodes to be processed in the current frame.
		rand_point_per_frame (list): A list of sets, where each set contains the random points to be processed in the current frame.
		trans_matrix (list): A transformation matrix used to convert coordinates from the original system to the image system.
		result (dict): A dictionary containing the results of the RRT* algorithm, including the history of nodes.

	Returns:
		list: A list of images, where each image represents the trees after processing a specific frame.
	"""
	result_frames_nodes = []
	result_frames_nodes.append(maze)
	for idx, nodes_set in enumerate(nodes_per_frame):
		plotted_nodes = result_frames_nodes[-1].copy()
		for idx_1, node in enumerate(nodes_set):
			#represent tree as a connection of lines
			node_image = coordinate_image(node, trans_matrix)
			#node_image_first_parent = coordinate_image(result['record']['history_nodes'][node]['parent'][0],trans_matrix)
			node_image_last_parent = coordinate_image(result['record']['history_nodes'][node]['parent'][0],trans_matrix)
			color_line = COLOR_TREE_A if node in result['tree_A'] else COLOR_TREE_B
			line = np.array([node_image_last_parent, node_image], np.int32)
			#draw line to parent
			cv2.line(plotted_nodes, (line[0][1],line[0][0]), (line[1][1],line[1][0]), color_line, 4)
			#draw random point
			y_rand, x_rand = coordinate_image(rand_point_per_frame[idx][idx_1], trans_matrix)
			cv2.circle(plotted_nodes, ( x_rand, y_rand ) , 3, COLOR_RAND_POINT, thickness=-1)
		result_frames_nodes.append(plotted_nodes)
	return result_frames_nodes

def draw_rewire(trees_done, total_nodes, result, trans_matrix):
	"""
	This function draws the rewiring process of the RRT* algorithm. It connects the current node to its parents,
	highlighting the rewiring process by coloring the lines differently.

	Args:
		trees_done (np.ndarray): The image of the trees after the exploration phase.
		total_nodes (list): A list of sets, where each set contains the nodes to be processed in the current frame.
		result (dict): A dictionary containing the results of the RRT* algorithm, including the history of nodes.
		trans_matrix (list): A transformation matrix used to convert coordinates from the original system to the image system.

	Returns:
		list: A list of images, where each image represents the trees after the rewiring process for a specific frame.
	"""
	result_rewire_nodes = []
	result_rewire_nodes.append(trees_done)
	for nodes_set in total_nodes:
		plotted_rewire = result_rewire_nodes[-1].copy()
		for node in nodes_set:
			#construct a set of lines which same origin the current node and end points the parents
			points_rewire = []
			lines_rewire = []
			node_image = coordinate_image(node, trans_matrix)
			parents_rewire = result['record']['history_nodes'][node]['parent'][1:]
			number_parents = len(parents_rewire)
			points_rewire = np.array( [ coordinate_image(point, trans_matrix) for point in parents_rewire], np.int32)
			lines_rewire = [ [node_image, point] for point in points_rewire ]
			for idx, line in enumerate(lines_rewire):
				color_line = COLOR_REWIRE if idx < number_parents-1 else COLOR_FINAL_PARENT
				thick_line = 8 if idx < number_parents-1 else 5
				cv2.line(plotted_rewire, (line[0][1],line[0][0]), (line[1][1],line[1][0]), color_line, thick_line)
				result_rewire_nodes.append(plotted_rewire)
	return result_rewire_nodes

def draw_goal_path(rewires_done, goal_lines_per_frame, color_path):
	"""
	Draws the goal path in the animation. Each path is represented as a set of connected lines.

	Args:
		rewires_done (np.ndarray): The image of the trees after the rewiring process.
		goal_lines_per_frame (list): A list of sets, where each set contains the lines to be drawn in the current frame.
		color_path (tuple): A tuple representing the color of the lines to be drawn.

	Returns:
		list: A list of images, where each image represents the goal path after processing a specific frame.
	"""
	result_frames_goal = []
	first_frame_goal = rewires_done.copy()
	for set_lines in goal_lines_per_frame:
		for line in set_lines:
			cv2.line(first_frame_goal, (line[0][1],line[0][0]), (line[1][1],line[1][0]), color_path, 10)
		result_frames_goal.append(first_frame_goal.copy())
	return result_frames_goal

def chunk_nodes(array_nodes):
	"""
	Divides an array of nodes into smaller chunks for animation purposes.

	Parameters:
	array_nodes (list): A list of nodes to be divided.

	Returns:
	list: A list of lists, where each sub-list represents a chunk of nodes.

	The function calculates the ratio of nodes per frame based on the maximum number of frames allowed (MAX_FRAME_NODES)
	and the minimum number of nodes per frame (NODES_PER_FRAME). It then divides the array of nodes into chunks of this size
	using the divide_array function. If the calculated ratio is less than the minimum number of nodes per frame, it sets
	the ratio to the minimum value.
	"""
	ratio_nodes_per_frame = len(array_nodes) // MAX_FRAME_NODES
	ratio_nodes_per_frame = ratio_nodes_per_frame if ratio_nodes_per_frame > NODES_PER_FRAME else NODES_PER_FRAME
	nodes_per_frame = divide_array(ratio_nodes_per_frame, array_nodes)
	return nodes_per_frame

def chunk_goal(path_total, trans_matrix):
	"""
	Divides the goal path into smaller chunks for animation purposes.

	Parameters:
	path_total (list): A list of tuples representing the coordinates of the goal path.
	trans_matrix (list): A transformation matrix used to convert coordinates from the original system to the image system.

	Returns:
	list: A list of lists, where each sub-list represents a chunk of lines to be drawn in the current frame.
	"""
	#goal_path
	goal_path_image = np.array([ coordinate_image(point, trans_matrix) for point in path_total ], np.int32)
	goal_path_lines = []
	for idx in range(len(goal_path_image)-1):
		goal_path_lines.append([goal_path_image[idx], goal_path_image[idx+1]])
	#Set lines per frame to display goal
	ratio_goal_per_frame = len(goal_path_lines) // MAX_GOAL_FRAMES
	ratio_goal_per_frame = ratio_goal_per_frame if ratio_goal_per_frame > LINES_PER_FRAME else LINES_PER_FRAME
	goal_lines_per_frame = divide_array(ratio_goal_per_frame, goal_path_lines)
	return goal_lines_per_frame

def resize_frames(frames, name):
	"""
	Resizes a list of frames to a specified scale factor using OpenCV's resize function.

	Parameters:
	frames (list): A list of numpy arrays representing the frames to be resized.
	name (str): A string representing the name of the frames, used for progress tracking.

	Returns:
	list: A list of numpy arrays representing the resized frames.

	The function uses OpenCV's resize function to resize each frame in the input list.
	The interpolation method used is INTER_LINEAR, which provides a good balance between speed and quality.
	The function uses tqdm to display a progress bar during the resizing process.
	"""
	final_resized_frames = []
	for frame in tqdm(frames,desc = f'Resizing {name}'):
		resized_frame = cv2.resize(frame, None, fx= SCALE_FACTOR, fy= SCALE_FACTOR, interpolation= cv2.INTER_LINEAR)
		final_resized_frames.append(resized_frame)
	return final_resized_frames

def processing_video(frames_nodes, frames_rewire_nodes, frames_paths, resize_width, resize_height):
	"""
	This function processes the frames of the animation, resizes them, and writes them to a video file.

	Parameters:
	frames_nodes (list): A list of numpy arrays representing the frames of the exploration space.
	frames_rewire_nodes (list): A list of numpy arrays representing the frames of the exploration rewire process.
	frames_paths (list): A list of lists, where each sub-list contains numpy arrays representing the frames of the path solution.
	resize_width (int): The width to which the frames should be resized.
	resize_height (int): The height to which the frames should be resized.

	Returns:
	None. The function writes the processed frames to a video file.
	"""
	#downsizing total images to a dimension for encoding video
	resize_frames_nodes = resize_frames(frames_nodes, 'exploration space frames')
	resize_rewire_nodes = resize_frames(frames_rewire_nodes, 'exploration rewire frames')
	resize_frames_path = []
	for path_frame in frames_paths:
		resize_frames_path += resize_frames(path_frame, 'path solution frames')
		#add extra frames for the end to display more time the final result
		extra_frames = []
		for idx in range(30):
			extra_frames.append(resize_frames_path[-1])
		resize_frames_path += extra_frames
	result_frames_total = resize_frames_nodes + resize_rewire_nodes + resize_frames_path #+ extra_frames
	try:
		video = cv2.VideoWriter(
					'simulation.mp4', cv2.VideoWriter_fourcc(*'mp4v'), FPS, (resize_width, resize_height))
		for frame in tqdm(result_frames_total, desc ="Creating video..."):
			video.write(frame)
		video.release()
	except Exception as err:
		print(err)
		print('Problem generation Video. Please check your dependencies and try again.')

def create_animation(init, goal, result, path_total, given_map,option, constraints):
	"""
	This function creates an animation of the RRT* algorithm's exploration and solution process.

	Parameters:
	init (tuple): The initial state of the robot.
	goal (tuple): The goal state of the robot.
	result (dict): The result of the RRT* algorithm, containing the history of nodes and other relevant information.
	path_total (tuple): A tuple containing the initial path, optimal path, smoothed path, and other relevant information.
	given_map (np.ndarray): An optional image of the maze or arena.
	option (int): An option to choose between different maze configurations.
	constraints (tuple): A tuple containing the border size, width, and height of the arena.

	Returns:
	None. The function creates an animation and writes it to a video file.
	"""
	_, WIDTH_SPACE, HEIGHT_SPACE = constraints
	TRANS_MATRIX = [ [0,-1, HEIGHT_SPACE],[1, 0, 0],[0, 0, 1] ] #from origin coord system to image coord system
	maze = create_maze(init, goal, given_map, option, TRANS_MATRIX, constraints)
	nodes_per_frame = chunk_nodes(result['record']['order_nodes'])
	rand_point_per_frame = chunk_nodes(result['record']['rand_samples'])
	frames_nodes = draw_trees(maze, nodes_per_frame, rand_point_per_frame, TRANS_MATRIX, result)
	frames_rewire_nodes = draw_rewire(frames_nodes[-1], nodes_per_frame, result, TRANS_MATRIX)
	init_path, opt_path, smth_path, _ = path_total
	goal_lines_per_frame = chunk_goal(init_path, TRANS_MATRIX)
	frames_goal = draw_goal_path(frames_rewire_nodes[-1], goal_lines_per_frame, COLOR_INITIAL)
	optimal_lines_per_frame = chunk_goal(opt_path, TRANS_MATRIX)
	frames_opt = draw_goal_path(frames_rewire_nodes[-1], optimal_lines_per_frame, COLOR_OPTIMAL)
	smooth_lines_per_frame = chunk_goal(smth_path, TRANS_MATRIX)
	frames_smooth = draw_goal_path(frames_rewire_nodes[-1], smooth_lines_per_frame, COLOR_SMOOTH)
	frames_path = (frames_goal, frames_opt, frames_smooth)
	RESIZE_WIDTH = int(WIDTH_SPACE * SCALE_FACTOR)
	RESIZE_HEIGHT = int(HEIGHT_SPACE * SCALE_FACTOR)
	processing_video(frames_nodes, frames_rewire_nodes, frames_path, RESIZE_WIDTH, RESIZE_HEIGHT)

# if __name__ == "__main__":
# 	create_simulation()