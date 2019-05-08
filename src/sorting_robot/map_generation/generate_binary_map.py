from generate_map_config import Cell, CellType, Direction, Turn
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.table import Table
import os

homeDir = os.environ['HOME']
MAP_CONFIG_FILE_LOCATION = homeDir + '/catkin_ws/src/sorting_robot/data/grid.npy'
BINARY_GRID_IMAGE_LOCATION = homeDir + '/catkin_ws/src/sorting_robot/data/binary_grid.png'
PIXEL_TO_CELL_RATIO = 50


def saveDataToImage(data, filename):
    def booleanToImage(data):
        size = data.shape[::-1]
        databytes = np.packbits(data, axis=1)
        return Image.frombytes(mode='1', size=size, data=databytes)

    im = booleanToImage(data)
    im.save(filename, "PNG")


def generateBinaryMap():
    try:
        grid = np.load(MAP_CONFIG_FILE_LOCATION)
        enlargedBinaryGrid = np.ones(
            (grid.shape[0] * PIXEL_TO_CELL_RATIO, grid.shape[1] * PIXEL_TO_CELL_RATIO), dtype=bool)

        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                if grid[row][col].isObstacle:
                    enlargedBinaryGrid[row * PIXEL_TO_CELL_RATIO: (
                        row + 1) * PIXEL_TO_CELL_RATIO, col * PIXEL_TO_CELL_RATIO: (col + 1) * PIXEL_TO_CELL_RATIO] = False

        saveDataToImage(enlargedBinaryGrid, BINARY_GRID_IMAGE_LOCATION)
    except IOError:
        print(MAP_CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == '__main__':
    generateBinaryMap()
