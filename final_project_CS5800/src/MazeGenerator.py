"""
    CS5800 - Algorithm - Lindsay Jamieson
    Topic: 3D maze - A* Path Finding Algorithm
    Group: Sida Zhang, Qiwei Hu, Hongyu Wan, Zheng Yin

    Generate the 3-D Maze
"""
import sys
import numpy as np


class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.dist = sys.maxsize

    def get_position(self):
        position = [self.x, self.y, self.z]
        return position


class TheMaze:
    def __init__(self, size):
        self.size = size
        self.obstacles_size = size // 2
        self.obstacles = []

        self.generate_obstacle()

    # check if a point is in obstacle
    def obstacle_check(self, x, y, z):
        for obs in self.obstacles:
            for i in obs:
                if x == i.x and y == i.y and z == i.z:
                    return True
        return False

    # generate obstacle randomly
    def obstacle(self, x_obstacle, y_obstacle, z_obstacle):
        block = []
        x = np.random.randint(0, self.size - x_obstacle)
        y = np.random.randint(0, self.size - y_obstacle)
        z = np.random.randint(0, self.size - z_obstacle)
        for i in range(x_obstacle):
            for j in range(y_obstacle):
                for k in range(z_obstacle):
                    block.append(Position(x + i, y + j, z + k))
        return block

    # append all the obstacles to the maze
    def generate_obstacle(self):
        for i in range(self.obstacles_size - 1):
            direction = np.random.randint(0, 2)

            if direction == 0:
                obstacle_size = np.random.randint(1, self.size)
                self.obstacles.append(self.obstacle(obstacle_size, obstacle_size, 1))
            elif direction == 1:
                obstacle_size = np.random.randint(1, self.size)
                self.obstacles.append(self.obstacle(obstacle_size, 1, obstacle_size))
            elif direction == 2:
                obstacle_size = np.random.randint(1, self.size)
                self.obstacles.append(self.obstacle(1, obstacle_size, obstacle_size))