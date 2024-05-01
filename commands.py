#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from caf_rrt_star import plan_caf_rrt_star
from solution import generate_path
import os
import time
import numpy as np
import math

WIDTH = 6000
HEIGHT = 2000
BORDER = 10

class GoalPath(Node):

	def __init__(self):
		super().__init__('goal_path')

		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.cmd_pos_subs = self.create_subscription(Odometry,'/odom', self.get_coord)
		self.twist_vel_msg = Twist()
		self.K_angle_ct = 0.5
		self.K_vel_ct = 0.5
		self.min_err_angle = 0.0174533
		self.min_err_pos = 0.0174533
		self.max_vel_lin = 0.26 * 1000 #mm/s
		self.pos_x = None
		self.pos_y = None
		self.yaw = None

	def get_coord(self, msg):
		#TODO: CHECK UNITS
		self.pos_x = msg.pose.pose.position.x *1000 # mm
		self.pos_y = msg.pose.pose.position.y *1000 #mm
		quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		_, _, yaw = euler_from_quaternion(quaternion)
		self.yaw = yaw #rad

	def controller(self,  start, end):
		# self.twist_vel_msg.linear.x = 0
		# self.twist_vel_msg.angular.z = 0
		x_start, y_start = start
		x_end, y_end = end
		slope = (y_end - y_start) / (x_end - x_start)
		goal_orientation = math.atan(slope)
		error_position = np.inf
		error_orientation = np.inf
		# #first settle just orientation
		# while (error_orientation > self.min_err_angle) and rclpy.ok():
		# 	error_orientation = goal_orientation - self.yaw
		# 	self.twist_vel_msg.angular.z = self.K_angle_ct * error_orientation
		# 	self.cmd_vel_pub.publish(self.twist_vel_msg)
		# #now settle orientation and linear velocity so it follow the path as expected
		self.twist_vel_msg.linear.x = 0
		self.twist_vel_msg.angular.z = 0
		while (error_position > self.min_err_pos) and rclpy.ok():
			error_orientation = goal_orientation - self.yaw
			# Calculate velocity based on the orientation difference
			velocity = self.max_vel_lin * (1 - self.K_vel_ct * error_orientation)
			# Ensure velocity is within limits
			self.twist_vel_msg.linear.x = max(0, min(velocity, self.max_vel_lin))
			self.twist_vel_msg.angular.z = self.K_angle_ct * error_orientation
			self.cmd_vel_pub.publish(self.twist_vel_msg)
			error_position = math.sqrt((self.pos_x - x_end)**2 + (self.pos_y - y_end)**2)

	def run_command_sequence(self, sequences):
		self.msg = """
		Being path planning and action Running CAF-RRT* algorithm
		---------------------------
		"""
		for initial, goal in sequences:
			result = plan_caf_rrt_star(initial, goal, (BORDER, WIDTH, HEIGHT))
			solution_path = generate_path(**result)
			for start,end in solution_path:
				self.controller(start, end)

def main(args=None):
	rclpy.init(args=args)
	# Get the current working directory
	current_directory = os.getcwd()
	additional_path = 'src/turtlebot3_project3/scripts/'
	full_path = os.path.join(current_directory, additional_path)
	print(full_path)
	file = open(f'{full_path}plan_set.txt', "r")
	data = file.readlines()
	file.close()
	target_values = [state.strip().replace('\n','') for state in data]
	pos_plan = []
	for command in target_values:
		x_pos,y_pos = command.split(',')
		pos_plan.append((int(x_pos), int(y_pos)))
	pos_sequences = []
	for idx in range(len(pos_plan)-1):
		pos_sequences.append([pos_plan[idx], pos_plan[idx+1]])
	node = GoalPath()
	node.run_command_sequence(pos_sequences)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()