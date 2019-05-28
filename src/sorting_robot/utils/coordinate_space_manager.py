import os
import math
import numpy as np
from ..map_generation.generate_map_config import Cell, Direction

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'


def directionToRadians(direction):
    if direction == Direction.LEFT:
        return math.radians(180)
    elif direction == Direction.RIGHT:
        return math.radians(0)
    elif direction == Direction.UP:
        return math.radians(90)
    elif direction == Direction.DOWN:
        return math.radians(270)


class CoordinateSpaceManager:
    def __init__(self):
        try:
            mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
        except IOError:
            print(CONFIG_FILE_LOCATION +
                  " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config");
            raise IOError("Config file doesn't exist.")
        else:
            self.numRowsInGrid = mapConfiguration['num_rows']
            self.numColumnsInGrid = mapConfiguration['num_columns']
            self.cellLength = mapConfiguration['cell_length_in_meters']
            self.grid = mapConfiguration['grid']

    def convertPointToCell(self, point):
        col = int(point[0] // self.cellLength)
        row = self.numRowsInGrid - int(point[1] // self.cellLength) - 1
        return row, col

    def convertCellToVector(self, cell):
        r, c = cell
        RANGE_Y = self.numRowsInGrid * self.cellLength
        # (x, y) is the mid point of the cell in world coordinates
        x = (c + 0.5) * self.cellLength
        y = (r + 0.5) * self.cellLength
        y = RANGE_Y - y
        if len(self.grid[r][c].directions) == 0:
            raise RuntimeError(
                "cell ({}, {}) doesn't have a direction.".format(r, c))
        # select any one direction from the list of valid directions arbitrarily
        theta = directionToRadians(list(self.grid[r][c].directions)[0])
        return (x, y, theta)

    def convertCellToPoint(self, cell):
        r, c = cell
        RANGE_Y = self.numRowsInGrid * self.cellLength
        x = (c + 0.5) * self.cellLength
        y = (r + 0.5) * self.cellLength
        y = RANGE_Y - y
        return (x, y)
