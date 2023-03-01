import numpy as np
import matplotlib.pyplot as plt


#Represents a position in the grid
class Cell():
	def __init__(self, x, y, z, cost_from_start=0, parent=None, heuristic=0):
		self.x = x
		self.y = y
		self.z = z
		self.cost_from_start = cost_from_start
		self.heuristic = heuristic
		self.total_cost = cost_from_start + heuristic
		self.parent = parent #previous cell in the path, this information is used to reconstruct the path at the end of the algorithm, when the stop cell is found

	#overriding the == operator to compare two cells by their coordinates
	def __eq__(self, other_cell):
		return self.x == other_cell.x and self.y == other_cell.y and self.z == other_cell.z
	


class PathFinder3D():
	def __init__(self):
		pass

	#distance between two cells
	def distance(self, cell1, cell2, distance_metric):
		dist = 0
		if(distance_metric == "euclidean"):
			dist = np.sqrt((cell1.x - cell2.x)**2 + (cell1.y - cell2.y)**2 + (cell1.z - cell2.z)**2)
		elif(distance_metric == "manhattan"):
			dist = abs(cell1.x - cell2.x) + abs(cell1.y - cell2.y) + abs(cell1.z - cell2.z)
		else:
			print("Unknown distance metric.")
		return dist


	#main function, finds a path from the start position to the stop cell in a 3d maze filled with 0 or 1's. Uses the A* algorithm and either euclidean or manhattan distances as heuristic.
	def find_path(self, maze_3d, start_coord, stop_coord, plot_maze=True, dot_size=200, plot_only_the_path=False, distance_metric="manhattan"):
		#checks if the start and stop cells are not out of bounds
		if(start_coord[0] < 0 or start_coord[0] >= maze_3d.shape[0] or start_coord[1] < 0 or start_coord[1] >= maze_3d.shape[1] or start_coord[2] < 0 or start_coord[2] >= maze_3d.shape[2]):
			print("Start cell is out of bounds.")
			return []
		if(stop_coord[0] < 0 or stop_coord[0] >= maze_3d.shape[0] or stop_coord[1] < 0 or stop_coord[1] >= maze_3d.shape[1] or stop_coord[2] < 0 or stop_coord[2] >= maze_3d.shape[2]):
			print("Stop cell is out of bounds.")
			return []
		
		#checks if the start or stop cells are not obstacles
		if(maze_3d[start_coord[0]][start_coord[1]][start_coord[2]] == 1 or maze_3d[stop_coord[0]][stop_coord[1]][stop_coord[2]] == 1):
			print("Start or stop cell is an obstacle, the maze is unsolvable.")
			return []


		startCell = Cell(start_coord[0], start_coord[1], start_coord[2])
		stopCell = Cell(stop_coord[0], stop_coord[1], stop_coord[2])

		openList = []
		closedList = []
		openList.append(startCell)
		
		path = []

		#loop of the A* algorithm
		while(len(openList) > 0):
			#openList stores the list of cells to be explored. The list gets sorted by total_cost (number of steps that it took
			#to get there + heuristic guess of the remaining distance to target), so that the cell with the lowest total cost is explored first.
			#When a cell is "explored", its valid neighbors are added to openList, and the algorithm keeps iterating like this until finding the stop cell.
			openList = sorted(openList, key=lambda cell: cell.total_cost, reverse=False)
			currentNode = openList.pop(0) #cell with lowest total cost

			#closedList stores the list of cells that have already been explored (cells that have been chosen at previous iterations of the loop and popped from openList)
			closedList.append(currentNode)

			if(currentNode == stopCell):
				#The stop cell has been found. We backtrack to reconstruct the path
				while(currentNode.parent is not None):
					path.insert(0, [currentNode.x, currentNode.y, currentNode.z])
					currentNode = currentNode.parent
				#adding the start cell's position too as required in the coding exercise's instructions
				path.insert(0, [start_coord[0], start_coord[1], start_coord[2]])
				print(f"Found a path ! It is {len(path)-1} steps long.")
				break
			
			# check all the neighbors. Neighbors are cells that are adjacent to the current cell. I assumed that diagonal steps are not allowed, it wasn't specified in the instructions.
			# first, we list all valid neighbors (we remove obstacles, out of bounds coordinates, and diagonal cells)
			neighbors = []
			for i in range(-1, 2):
				for j in range(-1, 2):
					for k in range(-1, 2):
						if(i != 0 or j != 0 or k != 0):
							#check that it's not diagonal cell (only one coordinate can change at a time)
							if((i != 0 and j != 0) or (i != 0 and k != 0) or (j != 0 and k != 0)):
								continue

							#check for out of bounds
							if(currentNode.x + i >= 0 and currentNode.x + i < maze_3d.shape[0] and currentNode.y + j >= 0 and currentNode.y + j < maze_3d.shape[1] and currentNode.z + k >= 0 and currentNode.z + k < maze_3d.shape[2]):
								#check if not an obstacle cell
								if(maze_3d[currentNode.x + i][currentNode.y + j][currentNode.z + k] == 0):
									neighbors.append(Cell(currentNode.x + i, currentNode.y + j, currentNode.z + k, currentNode.cost_from_start + 1, currentNode))
			
			#then, for each neighbors, we do additional verifications then add the valid ones to the list of cells to visit
			for neighbor in neighbors:
				#check if this position has not been already visited
				already_visited = False
				for closedCell in closedList:
					if(neighbor == closedCell):
						already_visited = True
						break
				if(not already_visited):
					#check if not already present in open list
					already_present = False
					for openCell in openList:
						if(neighbor == openCell):
							already_present = True
							break
					if(not already_present):
						#Cell is valid, we compute and set heuristic and total cost, then add to openList
						neighbor.heuristic = self.distance(neighbor, stopCell, distance_metric)
						neighbor.total_cost = neighbor.cost_from_start + neighbor.heuristic
						openList.append(neighbor)
		
		if(len(path) == 0):
			#no path has been found
			print("There is no possible path to the target cell, the maze is unsolvable.")

		if(plot_maze):
			self.plot_maze_and_solution(maze_3d, start_coord, stop_coord, path, dot_size, plot_only_the_path)
		
		return path
	

	#Plots the maze in a 3D scatter plot using matplotlib, and plots the path as a red line from the green starting cell to the yellow stop cell
	def plot_maze_and_solution(self, maze_3d, start, stop, path, dot_size=None, plot_only_the_path=False):
		if(not dot_size):
			dot_size = 200

		visualization_maze = maze_3d.copy()

		#mark the start and stop
		visualization_maze[start[0], start[1], start[2]] = 3
		visualization_maze[stop[0], stop[1], stop[2]] = 4

		# #marking the path
		# for coord in path[1:-1]:
		# 	visualization_maze[coord[0], coord[1], coord[2]] = 2

		fig = plt.figure()
		fig.suptitle('3D maze with path, from the green starting point to the yellow target cell', fontsize=16)
		ax = fig.add_subplot(111, projection='3d')

		#plotting the maze with a scatter plot where each point has a color corresponding to the maze's cell value
		if(not plot_only_the_path):
			ax.scatter(*np.where(visualization_maze==0), c='lightblue', s=dot_size)
			ax.scatter(*np.where(visualization_maze==1), c='black', s=dot_size)
			# ax.scatter(*np.where(visualization_maze==2), c='red', s=dot_size)
		ax.scatter(*np.where(visualization_maze==3), c='green', s=dot_size)
		ax.scatter(*np.where(visualization_maze==4), c='yellow', s=dot_size)

		# plotting the path as a line
		xs = [coord[0] for coord in path]
		ys = [coord[1] for coord in path]
		zs = [coord[2] for coord in path]
		ax.plot(xs, ys, zs, c='red')
		
		plt.show()