"""
    CS5800 - Algorithm - Lindsay Jamieson
    Topic: 3D maze - A* Path Finding Algorithm
    Group: Sida Zhang, Qiwei Hu, Hongyu Wan, Zheng Yin

    execute the program and draw the 3-D maze with MatPlot
    3D projection:
"""
import AStarAlgorithm
import MazeGenerator
import numpy as np
import matplotlib.pyplot as plt
# This import registers the 3D projection on Jupyter, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D


# draw obstacle in red cubes
def obstacle_cube(position):
    n_cubes[position[0]][position[1]][position[2]] = True


# draw path in green cubes
def find_path_3d(position):
    path_cubes[position[0]][position[1]][position[2]] = True


# build up the numpy logo
def explode(data):
    # https://matplotlib.org/3.5.0/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html
    size = np.array(data.shape)*2
    data_e = np.zeros(size - 1, dtype=data.dtype)
    data_e[::2, ::2, ::2] = data
    return data_e


def plot_now(n_cubes, path_cubes):
    # red
    # 26 15%
    # 4D 30%
    # for transparency: https://css-tricks.com/8-digit-hex-codes/
    facecolors = np.where(n_cubes, '#d370704D', '#d3707003')
    edgecolors = np.where(n_cubes, '#BFAB6E', '#7D84A60D')
    facecolors_3 = np.where(path_cubes, '#5ccd954D', '#5ccd9503')
    edgecolors_3 = np.where(path_cubes, '#58b445', '#7D84A60D')
    filled = np.ones(n_cubes.shape)

    # upscale the above voxel image, leaving gaps
    filled_2 = explode(filled)
    fcolors_2 = explode(facecolors)
    ecolors_2 = explode(edgecolors)

    fcolors_3 = explode(facecolors_3)
    ecolors_3 = explode(edgecolors_3)

    # Shrink the gaps
    x, y, z = np.indices(np.array(filled_2.shape) + 1).astype(float) // 2
    x[0::2, :, :] += 0.05
    y[:, 0::2, :] += 0.05
    z[:, :, 0::2] += 0.05
    x[1::2, :, :] += 0.95
    y[:, 1::2, :] += 0.95
    z[:, :, 1::2] += 0.95

    fig = plt.figure()
    # ax = fig.gca(projection='3d')
    ax = Axes3D(fig)
    ax.voxels(x, y, z, filled_2, facecolors=fcolors_2, edgecolors=ecolors_2)
    ax.voxels(x, y, z, filled_2, facecolors=fcolors_3, edgecolors=ecolors_3)
    plt.axis('off')
    plt.show()


if __name__ == '__main__':
    print("A* Maze Generator has started")
    val = input("Please enter the size of maze you want to make (> 5): ")
    if int(val) <= 5:
        print("Please enter size larger than 5x5x5 for the 3D maze.")
        exit()
    maze = MazeGenerator.TheMaze(int(val))
    n_cubes = np.zeros((int(val), int(val), int(val)), dtype=bool)
    path_cubes = np.zeros((int(val), int(val), int(val)), dtype=bool)
    for a in maze.obstacles:
        for b in a:
            obstacle_cube(b.get_position())

    a_star = AStarAlgorithm.AStarAlgo(maze)
    path = a_star.execute()

    print("A path has found!")
    for i in path:
        print("[" + str(i.x) + ", " + str(i.y) + ", " + str(i.z) + "]", end=' - ')
        find_path_3d(i.get_position())
    print("\n")
    plot_now(n_cubes, path_cubes)
    print("A* Maze Generator is now completed, the path is now shown in MatPlot")
