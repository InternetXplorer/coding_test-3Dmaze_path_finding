import numpy as np
from numpy import array
from path_finder import PathFinder3D
import random


seed = random.randint(0, 1000)
np.random.seed(seed)

#random maze generation and random start and stop positions
maze_3d = np.random.randint(low=0, high=2, size=(5,5,5))
start = [np.random.randint(low=0, high=maze_3d.shape[0]), np.random.randint(low=0, high=maze_3d.shape[1]), np.random.randint(low=0, high=maze_3d.shape[2])]
stop = [np.random.randint(low=0, high=maze_3d.shape[0]), np.random.randint(low=0, high=maze_3d.shape[1]), np.random.randint(low=0, high=maze_3d.shape[2])]
#remove obstacles from start and stop cells to avoid having to generate a new maze every time the maze is unsolvable for this reason
maze_3d[start[0]][start[1]][start[2]] = 0
maze_3d[stop[0]][stop[1]][stop[2]] = 0

print(maze_3d)
print(f"Maze shape : {maze_3d.shape}")
print(f"Seed: {seed}")
print(f"Start : {start}")
print(f"Stop : {stop}")


#PATH FINDING
path_finder = PathFinder3D()
path = path_finder.find_path(maze_3d, start, stop, plot_maze=True, dot_size=200, plot_only_the_path=False, distance_metric="manhattan")
print(path)