from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import from_levels_and_colors
import sys
import os
from random import randint

class NLinkArm(object):
	"""
	Class for controlling and plotting a planar arm with an arbitrary number of links.
	"""

	def __init__(self, link_lengths, joint_angles):
		 self.n_links = len(link_lengths)
		 if self.n_links != len(joint_angles):
		 	raise ValueError()

		 self.link_lengths = np.array(link_lengths)
		 self.joint_angles = np.array(joint_angles)
		 self.points = [[0, 0] for _ in range(self.n_links + 1)]

		 self.lim = sum(link_lengths)
		 self.update_points()

	def update_joints(self, joint_angles):
		 self.joint_angles = joint_angles
		 self.update_points()

	def update_points(self):
		 for i in range(1, self.n_links + 1):
			 self.points[i][0] = self.points[i - 1][0] + self.link_lengths[i - 1] * np.cos(np.sum(self.joint_angles[:i]))
			 self.points[i][1] = self.points[i - 1][1] + self.link_lengths[i - 1] * np.sin(np.sum(self.joint_angles[:i]))

		 self.end_effector = [self.points[self.n_links][0],self.points[self.n_links][1]]

	def plot_arm(self, myplt, obstacles=[]):
		for obstacle in obstacles:
			circle = myplt.Circle(
			(obstacle[0], obstacle[1]), radius= obstacle[2], fc='k')
			myplt.gca().add_patch(circle)

		for i in range(self.n_links + 1):
			if i is not self.n_links:
				myplt.plot([self.points[i][0], self.points[i + 1][0]],[self.points[i][1], self.points[i + 1][1]], 'r-')
				myplt.plot(self.points[i][0], self.points[i][1], 'k.')

				myplt.draw()



def animate(grid, arm, route, obstacles, M, start_pos, goal_pos, start, goal):
	 fig, axs = plt.subplots(1, 2)
	 colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
	 levels = [0, 1, 2, 3, 4, 5, 6, 7]
	 cmap, norm = from_levels_and_colors(levels, colors)
	 traj = [[start_pos[0], start_pos[1]] for _ in range(len(route) + 2)]

	 grid_imshow = grid.T
	 for i, node in enumerate(route):
		 plt.subplot(1, 2, 1)
		 grid[node] = 6
		 plt.cla()
		 plt.imshow(grid_imshow, cmap=cmap, norm=norm, interpolation=None)
		 theta1 = 2 * pi * node[0] / M - pi
		 theta2 = 2 * pi * node[1] / M - pi
		 arm.update_joints([theta1, theta2])
		 traj[i+1] = [arm.end_effector[0],arm.end_effector[1]]
		 plt.scatter(start[0],start[1],marker='o',c='yellow')
		 plt.scatter(goal[0],goal[1],marker='o',c='green')
		 plt.xlim(0,M)
		 plt.ylim(0,M)
		 plt.xticks([])
		 plt.yticks([])
		 plt.axis('equal')
		 plt.xlabel('-pi                                    pi\ntheta 1')
		 plt.ylabel('theta 2\n-pi                                    pi')

		 plt.subplot(1, 2, 2)
		 plt.cla()
		 plt.xlim(-np.sum(arm.link_lengths)-0.5, np.sum(arm.link_lengths)+0.5)
		 plt.ylim(-np.sum(arm.link_lengths)-0.5, np.sum(arm.link_lengths)+0.5)
		 plt.axis('equal')
		 plt.xlabel('X(m)')
		 plt.ylabel('Y(m)')

		 plt.scatter(start_pos[0],start_pos[1],marker='o',c='yellow')
		 plt.scatter(goal_pos[0],goal_pos[1],marker='o',c='green')
		 for j in range(i+1):
		 	plt.plot([traj[j][0], traj[j+1][0]] ,[traj[j][1], traj[j+1][1]], 
			c='orange',linestyle='--',dashes=(5,1))
		 
		 arm.plot_arm(plt, obstacles=obstacles)

		 plt.show()
		 plt.pause(0.1)

		 print("\tstep{}: end effector={} ".format(i,traj[i+1]))
		 print("\t        joint angles={})".format([theta1,theta2]))

def angle_to_grid_index(joint_angles, M):
	 index_x = int(joint_angles[0] // (2*pi/M) + M // 2)
	 index_y = int(joint_angles[1] // (2*pi/M) + M // 2)
	 return (index_x, index_y)

def grid_index_to_angle(indices, M):
	 js_1 = float((indices[0] - M // 2)*2*pi/M)
	 js_2 = float((indices[1] - M // 2)*2*pi/M)
	 return (js_1, js_2)

plt.ion()
