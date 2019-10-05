from NLinkArm import *

def main():
	 # step 0: simulation parameters
	M = 100	 
	link_length = [0.5, 1.5]
	initial_link_angle = [0, 0]
	obstacles = [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.7]]
	goal_found = False

	 # step 1: robot and environement setup
	arm = NLinkArm(link_length, initial_link_angle)
	# arm.n_links = len(link_length)
	# arm.link_lengths = np.array(link_lengths) [0.5,1.5]
	# arm.joint_angles = np.array(joint_angles) [0,0]
	# arm.points = [[0, 0], [0.5, 0.0], [2.0, 0.0]]

	grid = get_occupancy_grid(arm, obstacles, M)
	print("\nstep 1: robot and environment ready.")

	while not goal_found:
		# step 2: set start and random goal
		start = angle_to_grid_index(initial_link_angle,M)
		#goal = (randint(0,M),randint(0,M))
		goal = (50,99)
		start_js = grid_index_to_angle(start, M)
		goal_js = grid_index_to_angle(goal, M)
		goal_pos = forward_kinematics(link_length, goal_js)
		start_pos = forward_kinematics(link_length, start_js)
		if not (grid[start[0]][start[1]] == 0):
			print("Start pose is in collision with obstacles. Close system.") 
			break

		elif (grid[goal[0]][goal[1]] == 0):
			print("\nstep 2: \n\tstart_js  = {}, goal_js  = {}".format(start_js, goal_js))
			print("\tstart_pos = {}, goal_pos = {}".format(start_pos, goal_pos))
			goal_found = True

	if(goal_found):
		# step 3: motion planning
		#print('arm.points=',arm.points)
		route = astar_search(grid, start, goal, M)
		print("\nstep 3: motion planning completed.")

		# step 4: visualize result	
		print("\nstep 4: start visualization.")
		if len(route) >= 0:
			animate(grid, arm, route, obstacles, M, start_pos, goal_pos, start, goal)

		print("\nGoal reached.")

def forward_kinematics(link_length, joint_angles):
	 posx = link_length[0]*np.cos(joint_angles[0])+link_length[1]*np.cos(np.sum(joint_angles))
	 posy = link_length[0]*np.sin(joint_angles[0])+link_length[1]*np.sin(np.sum(joint_angles))
	 return (posx,posy)
	 
def inverse_kinematics(posx, posy, link_length):
	 goal_th = atan2(posy,posx)
	 A = link_length[0]
	 B = sqrt(posx*posx+posy*posy)
	 C = link_length[1]

	 C_th = np.arccos(float((A*A + B*B - C*C)/(2.0*A*B))) 
	 B_th = np.arccos(float((A*A + C*C - B*B)/(2.0*A*C))) 
	 theta1_sol1 = simplify_angle(goal_th + C_th)
	 theta2_sol1 = simplify_angle(-pi + B_th)
	 theta1_sol2 = simplify_angle(goal_th - C_th)
	 theta2_sol2 = simplify_angle(pi - B_th)

	 return [[theta1_sol1, theta2_sol1],[theta1_sol2, theta2_sol2]] 

def detect_collision(line_seg, circle):
	# TODO: Return True if the line segment is intersecting with the circle
	#       Otherwise return false.
	# 	 (1) line_seg[0][0], line_seg[0][1] is one point (x,y) of the line segment
	#	 (2) line_seg[1][0], line_seg[1][1] is another point (x,y) of the line segment
	#	 (3) circle[0],circle[1],circle[2]: the x,y of the circle center and the circle radius
	# 	 Hint: the closest point on the line segment should be greater than radius to be collision free.
	#       Useful functions: np.linalg.norm(), np.dot()
    x_vec=line_seg[1][0]-line_seg[0][0]
    y_vec=line_seg[1][1]-line_seg[0][1]
    vec_len=np.sqrt(x_vec**2+y_vec**2)
    unit_vec = [x_vec/vec_len,y_vec/vec_len]
    point_to_circle = [circle[0]-line_seg[0][0],circle[1]-line_seg[0][1]]
    projection=np.dot(point_to_circle,unit_vec)
    #projection_len=np.sqrt(projection[0]**2+projection[1]**2)
    #next_point_in_line = [line_seg[0][0]+projection[0],line_seg[0][1]+projection[1]]
    if projection < 0:
    	closest_point = [line_seg[0][0],line_seg[0][1]] 
    elif projection > vec_len:
    	closest_point = [line_seg[1][0],line_seg[1][1]]
    else :
    	projection_vec = [projection*i  for i in unit_vec]
    	closest_point = [sum(x) for x in zip(line_seg[0], projection_vec)]
    	#closest_point = [line_seg[0][0]+projection_vec[0],line_seg[] 
    dist_circle_line = np.sqrt((circle[0]-closest_point[0])**2+(circle[1]-closest_point[1])**2)
    if dist_circle_line > circle[2]:
    	return False
    else:
    	return True


def get_occupancy_grid(arm, obstacles, M):
	#obstacles = [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.7]]
	grid = [[0 for _ in range(M)] for _ in range(M)]
	theta_list = [2 * i * pi / M for i in range(-M // 2, M // 2 + 1)]
	#print(grid)
	#print(theta_list)
	for i in range(M):
		for j in range(M):

			# TODO: traverse through all the grid vertices, i.e. (theta1, theta2) combinations
			# 	 Find the occupacy status of each robot pose.
			# 	 Useful functions/variables: arm.update_joints(), arm.points, theta_list[index]

			# points = # somthing...
			goal_joint_angles = np.array([theta_list[i], theta_list[j]])
			arm.update_joints(goal_joint_angles)
			points = arm.points
			collision_detected = False
			for k in range(len(points) - 1):
				for obstacle in obstacles:
					# TODO: define line_seg and detect collisions
					# 	 Useful functions/variables: detect_collision(), points[index]
					line_seg = [points[k],points[k+1]]
					collision_detected = detect_collision(line_seg, obstacle)
					# line_seg = [something,something]
					# collision_detected = ?

					if collision_detected:
						break
				if collision_detected:
					break
			grid[i][j] = int(collision_detected)
	return np.array(grid)


#grid = 
def astar_search(grid, start_node, goal_node, M):
	colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
	levels = [0, 1, 2, 3, 4, 5, 6, 7]
	cmap, norm = from_levels_and_colors(levels, colors)

	grid[start_node] = 4
	grid[goal_node] = 5



	parent_map = [[() for _ in range(M)] for _ in range(M)]

	heuristic_map = calc_heuristic_map(M, goal_node)

	evaluation_map = np.full((M, M), np.inf)
	distance_map = np.full((M, M), np.inf)
	evaluation_map[start_node] = heuristic_map[start_node]
	distance_map[start_node] = 0
	while True:
		grid[start_node] = 4
		grid[goal_node] = 5

		current_node = np.unravel_index(np.argmin(evaluation_map, axis=None), evaluation_map.shape)
		min_distance = np.min(evaluation_map)

		if (current_node == goal_node) or np.isinf(min_distance):
			break

		grid[current_node] = 2
		evaluation_map[current_node] = np.inf

		i, j = current_node[0], current_node[1]
		#print('arm.points=',NLinkArm.points)
		neighbors = find_neighbors(i, j, M)
		print('neighbors=',neighbors)
		for neighbor in neighbors:
			if grid[neighbor] == 0 or grid[neighbor] == 5 or grid[neighbor] == 3:
				evaluation_map[neighbor] = min(evaluation_map[neighbor],heuristic_map[neighbor]+distance_map[current_node]+1)
				if evaluation_map[current_node]>heuristic_map[neighbor]+distance_map[current_node]+1:
					parent_map[neighbor[0]][neighbor[1]]=current_node
				distance_map[neighbor]=distance_map[current_node]+1
				grid[neighbor]=3
				# TODO: Update the score in the following maps
				#	 (1) evaluation_map[neighbor]
				#	 (2) distance_map[neighbor]:  update distance using distance_map[current_node]
				#	 (3) parent_map[neighbor]:    set to current node
				# 	 (4) grid[neighbor]: 	      set value to 3
				# 	 Update criteria: new evaluation_map[enighbor] value is decreased
	if np.isinf(evaluation_map[goal_node]):
		route = []
		print("No route found.")
	else:
		route = [goal_node]
		while parent_map[route[0][0]][route[0][1]] != ():
			# TODO: find the optimal route based on your exploration result
			#       Useful functions: 
			#	(1) route.insert(index, element): to add new node to route
			#	(2) parent_map[index1][index2]: find the parent of the node at grid coordinates (index1,index2)
			# 	(3) route[0][0], route[0][1] to access the grid coordinates of a node
			# route.insert(something...)
			index1,index2=parent_map[route[0][0]][route[0][1]]
			route.insert(0,(index1,index2))
		print("The route found covers %d grid cells." % len(route))
	return route


def find_neighbors(i, j, M):
	neighbors = []
	ileft = i-1
	iright = i+1
	jlow = j-1
	jhigh = j+1
	if (i == 0):
		ileft = M-1
	if (i == M-1):
		iright=0
	if (j==0):
		jlow = M-1
	if (j==M-1):
		jhigh=0
	neighbors.append((ileft,j))
	neighbors.append((iright,j))
	neighbors.append((i,jhigh))
	neighbors.append((i,jlow))
	# TODO: add the four neighbor nodes to the neighbor list
	# 	 grid index: i goes from 0 to M-1 (M possible values)
	# 	 grid index: j goes from 0 to M-1 (M possible values)

	# at i=0, theta1 = -pi
	# at j=0, theta2 = -pi
	# at i=M-1, theta1 = pi
	# at j=M-1, theta2 = pi

	# So be aware of the rounding effect in finding the neighbors at border nodes.
	# Useful function: neighbors.append((index1, index2)), where index1, index2 are the grid indices of it's neighbors
	
	#neighbors.append((index1,index2))
	return neighbors


def calc_heuristic_map(M, goal_node):
	 X, Y = np.meshgrid([i for i in range(M)], [i for i in range(M)])
	 #heuristic_map = 1*np.sqrt((X-goal_node[1])**2+(Y-goal_node[0])**2)
	 heuristic_map = 0.1*(np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0]))
	 print("heuristic_map=",heuristic_map)
	 for i in range(heuristic_map.shape[0]):
		 for j in range(heuristic_map.shape[1]):
			 heuristic_map[i, j] = min(heuristic_map[i, j],
			 i + 1 + heuristic_map[M - 1, j],
			 M - i + heuristic_map[0, j],
			 j + 1 + heuristic_map[i, M - 1],
			 M - j + heuristic_map[i, 0]
			 )
	 #print('heuristic_map=',heuristic_map)
	 return heuristic_map

def simplify_angle(angle):
	if angle > pi:
		angle -= 2*pi
	elif angle < -pi:
		angle += 2*pi
	return angle
if __name__ == '__main__':
	 main()
