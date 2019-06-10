import os
import math
import numpy as np
from ..map_generation.generate_map_config import Cell, Direction
from robot_info import RobotInfo

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}_configuration.npy'

# percent of the cell length to be added at each boundary
# if 10% is used, 0.9 to 0.1 will be counted as both 0+ and 1+ and 1.0 to 1.1 will be counted as both 1+ and 0+
BOUNDARY_CROSSING_TOLERANCE_PERCENT = 40

# tolerance to be used (in degrees) while converting degrees to direction
# if tolerance is 10 degrees, then UP refers to angles between 80 and 100 exclusive
ORIENTATION_TOLERANCE = 10


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
    def __init__(self, mapName='map'):
        global CONFIG_FILE_LOCATION
        CONFIG_FILE_LOCATION = CONFIG_FILE_LOCATION.format(mapName)
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

    # input: x, y, theta
    # theta is in radians, can correspond to any value between -180 and 180 degrees
    def convertPointToState(self, point):
        x, y, theta = point[0], point[1], point[2]
        col = int(x // self.cellLength)
        row = self.numRowsInGrid - int(y // self.cellLength) - 1

        theta = math.degrees(theta) if(theta>0) else math.degrees(theta)+360
        # check cyclic cases properly
        direction = None
        if 360 - ORIENTATION_TOLERANCE < theta <= 360 and 0 <= theta < 0 + ORIENTATION_TOLERANCE:
            direction = 0
        elif 90 - ORIENTATION_TOLERANCE < theta < 90 + ORIENTATION_TOLERANCE:
            direction = 90
        elif 180 - ORIENTATION_TOLERANCE < theta < 180 + ORIENTATION_TOLERANCE:
            direction = 180
        elif 270 - ORIENTATION_TOLERANCE < theta < 270 + ORIENTATION_TOLERANCE:
            direction = 270
        return row, col, direction

    # the input point is the mid point of the robot in world coordinates
    def convertPointToCells(self, point):
        x, y = point[0], point[1]
        col = int(x // self.cellLength)
        row = self.numRowsInGrid - int(y // self.cellLength) - 1
        lowerLeftPoint = self.getLowerLeftCornerWorldCoordinate((row, col))

        BOUNDARY_CROSSING_TOLERANCE_LENGTH = BOUNDARY_CROSSING_TOLERANCE_PERCENT / 100 * self.cellLength
        X_LOWER_LIMIT = lowerLeftPoint[0] + BOUNDARY_CROSSING_TOLERANCE_LENGTH
        X_UPPER_LIMIT = lowerLeftPoint[0] + self.cellLength - BOUNDARY_CROSSING_TOLERANCE_LENGTH
        Y_LOWER_LIMIT = lowerLeftPoint[1] + BOUNDARY_CROSSING_TOLERANCE_LENGTH
        Y_UPPER_LIMIT = lowerLeftPoint[1] + self.cellLength - BOUNDARY_CROSSING_TOLERANCE_LENGTH
        robotRadius = RobotInfo.getRobotRadiusInMeters()

        cells = []
        cells.append((row, col))
        if x - robotRadius < X_LOWER_LIMIT:
            if col - 1 >= 0:
                cells.append((row, col - 1))
        if x + robotRadius > X_UPPER_LIMIT:
            if col + 1 < self.numColumnsInGrid:
                cells.append((row, col + 1))
        if y - robotRadius < Y_LOWER_LIMIT:
            if row + 1 < self.numRowsInGrid:
                cells.append((row + 1, col))
        if y + robotRadius > Y_UPPER_LIMIT:
            if row - 1 >= 0:
                cells.append((row - 1, col))
        return cells

    # returns the coordinates of the mid point of the cell in world coordinates
    # direction is set to some arbitrary valid direction if none given
    # if no valid direction available, it is set to None
    def getWorldCoordinateWithDirection(self, cell):
        r, c = cell[0], cell[1]
        theta = None
        if len(cell) > 2:
            theta = math.radians(cell[2])
        else:
            if len(self.grid[r][c].directions) > 0:
                # select any one direction from the list of valid directions arbitrarily
                theta = directionToRadians(list(self.grid[r][c].directions)[0])

        # (x, y) is the mid point of the cell in world coordinates
        x = (c + 0.5) * self.cellLength
        y = (r + 0.5) * self.cellLength
        RANGE_Y = self.numRowsInGrid * self.cellLength
        y = RANGE_Y - y

        return (x, y, theta)

    # returns the coordinates of the lower left corner of the cell in world coordinates
    def getLowerLeftCornerWorldCoordinate(self, cell):
        r, c = cell
        x = c * self.cellLength
        y = r * self.cellLength
        RANGE_Y = self.numRowsInGrid * self.cellLength
        y = RANGE_Y - y - self.cellLength
        return (x, y)
