import os
import numpy as np
from PIL import Image
from generate_map_config import Cell, CellType, Direction, Turn
from ..utils.map_information_provider import CONFIG_FILE_LOCATION, BINARY_GRID_IMAGE_LOCATION, BINARY_GRID_IMAGE_LOCATION_STDR


def saveDataToImage(data, filename):
    def booleanToImage(data):
        size = data.shape[::-1]
        databytes = np.packbits(data, axis=1)
        return Image.frombytes(mode='1', size=size, data=databytes)

    im = booleanToImage(data)
    im.save(filename, "PNG")


def generateBinaryMap(pixelToCellRatio):
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")
    else:
        grid = mapConfiguration['grid']
        enlargedBinaryGrid = np.ones(
            (grid.shape[0] * pixelToCellRatio, grid.shape[1] * pixelToCellRatio), dtype=bool)

        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                if grid[row][col].isObstacle:
                    enlargedBinaryGrid[row * pixelToCellRatio: (
                        row + 1) * pixelToCellRatio, col * pixelToCellRatio: (col + 1) * pixelToCellRatio] = False

        saveDataToImage(enlargedBinaryGrid, BINARY_GRID_IMAGE_LOCATION)
        saveDataToImage(enlargedBinaryGrid, BINARY_GRID_IMAGE_LOCATION_STDR)


if __name__ == '__main__':
    generateBinaryMap('map', 50)
