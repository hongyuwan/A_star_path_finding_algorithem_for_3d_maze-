"""
    CS5800 - Algorithm - Lindsay Jamieson
    Topic: 3D maze - A* Path Finding Algorithm
    Group: Sida Zhang, Qiwei Hu, Hongyu Wan, Zheng Yin

    Executing A * Algorithm
"""
import sys
import numpy as np
import MazeGenerator


class AStarAlgo:
    def __init__(self, maze):
        self.maze = maze
        self.open_array = []
        self.close_array = []

    # validation check 1； check if the position is in the maze, and it is not an obstacle
    def validation(self, x, y, z):
        if x < 0 or y < 0 or z < 0:
            return False
        elif x >= self.maze.size or y >= self.maze.size or z >= self.maze.size:
            return False
        # check if the position is in obstacle
        return not self.maze.obstacle_check(x, y, z)

    # validation check 2: check if the coordinate is in the set
    def array_check(self, position, array):
        for pos in array:
            if position.x == pos.x and position.y == pos.y and position.z == pos.z:
                return True
        return False

    # validation check 3: check if the coordinate is in the open array
    def in_open_array(self, position):
        return self.array_check(position, self.open_array)

    # validation check 4: check if the coordinate is in the closed array
    def in_close_array(self, position):
        return self.array_check(position, self.close_array)

    # validation check 5: check if the coordinate is at the starting point
    def validation_start(self, position):
        if position.x == 0 and position.y == 0 and position.z == 0:
            return True
        return False

    # validation check 6: check if the coordinate is at the ending point
    def validation_end(self, position):
        if position.x == self.maze.size - 1 and position.y == self.maze.size - 1 and position.z == self.maze.size - 1:
            return True
        return False

    # returns the heuristic  distance in one of the following distance calculation methods
    def heuristic_dist(self, method, dx, dy, dz, dist):
        # method 0 returns the distance of Diagonal
        # method 1 returns the distance of Euclidean
        # method 2 returns the distance from Manhattan
        if method == "0":       # Diagonal
            return dx + dy + dz + (dist - 2) * min(dx, dy, dz)
        elif method == "1":     # Euclidean
            return np.sqrt(dx * dx + dy * dy + dz * dz)
        elif method == "2":     # Manhattan
            return dx + dy + dz
        return 0

    # returns the total distance from starting point to the ending point by averaging the three methods.
    def total_dist(self, position):
        distance = np.sqrt(3)
        sx = position.x
        sy = position.y
        sz = position.z

        ex = self.maze.size - 1 - position.x
        ey = self.maze.size - 1 - position.y
        ez = self.maze.size - 1 - position.z

        start_dist = (self.heuristic_dist("0", sx, sy, sz, distance) + self.heuristic_dist("1", sx, sy, sz, distance) +
                      self.heuristic_dist("2", sx, sy, sz, distance)) / 3
        end_dist = (self.heuristic_dist("0", ex, ey, ez, distance) + self.heuristic_dist("1", ex, ey, ez, distance) +
                    self.heuristic_dist("2", ex, ey, ez, distance)) / 3

        return start_dist + end_dist

    # return the shortest distance from the open array
    def compare(self):
        idx = 0
        current = -1
        min_dist = sys.maxsize
        for pos in self.open_array:
            dist = self.total_dist(pos)
            if dist < min_dist:
                min_dist = dist
                current = idx
            idx += 1
        return current

    # append neighbors to the open array
    def append_neighbor(self, x, y, z, parent):
        pos = MazeGenerator.Position(x, y, z)
        if not self.validation(x, y, z):
            return
        if self.in_close_array(pos):
            return
        if not self.in_open_array(pos):
            pos.parent = parent
            pos.dist = self.total_dist(pos)
            self.open_array.append(pos)

    # building the path
    def build_path(self, pos):
        path = []
        while True:
            path.insert(0, pos)
            if self.validation_start(pos):
                break
            else:
                pos = pos.parent
        return path

    # main function to run the A star algorithm.
    def execute(self):
        # initialize the starting point and distance
        start_point = MazeGenerator.Position(0, 0, 0)
        start_point.dist = 0
        self.open_array.append(start_point)

        while True:
            idx = self.compare()
            if idx < 0:
                print('No path found.')
                return []
            pos = self.open_array[idx]

            if self.validation_end(pos):
                return self.build_path(pos)

            del self.open_array[idx]
            self.close_array.append(pos)

            # Coordinates
            x = pos.x
            y = pos.y
            z = pos.z
            # Neighbors:
            value = [-1, 0, 1]
            for i in value:
                for j in value:
                    for k in value:
                        self.append_neighbor(x + i, y + j, z + k, pos)