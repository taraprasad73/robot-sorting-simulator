import numpy as np
from ..map_generation.generate_map_config import Cell, Direction, Turn, CellType
from random import shuffle
import math
import argparse
import os

homeDir = os.environ['HOME']
MAP_CONFIG_FILE_LOCATION = homeDir + '/catkin_ws/src/sorting_robot/data/grid.npy'
CELL_LENGTH = 0.5  # in meters
ADD_ROBOT_TEMPLATE = 'rosrun stdr_robot robot_handler add ~/catkin_ws/src/stdr_test/robots/pandora_robot.yaml {} {} {}\n'
GENERATED_SCRIPT_FILE = homeDir + '/catkin_ws/src/sorting_robot/data/spawn_robots.sh'


def parseArgs():
    parser = argparse.ArgumentParser(
        description="parameters for generating spawn locations")
    parser.add_argument("n", type=int, help="total number of robots")
    args = parser.parse_args()
    return args.n


def getRandomFreePoints(count, cells, grid):
    shuffle(cells)
    result = []
    for (r, c) in cells:
        if len(result) == count:
            break
        if not grid[r][c].isObstacle:
            result.append((r, c))
    return result


def directionToRadians(direction):
    if direction == Direction.LEFT:
        return math.radians(180)
    elif direction == Direction.RIGHT:
        return math.radians(0)
    elif direction == Direction.UP:
        return math.radians(90)
    elif direction == Direction.DOWN:
        return math.radians(270)


def convertCellsToCoordinates(freeCells, grid):
    result = []
    RANGE_Y = grid.shape[0] * CELL_LENGTH
    for (r, c) in freeCells:
        x = (c + 0.5) * CELL_LENGTH
        y = (r + 0.5) * CELL_LENGTH
        y = RANGE_Y - y
        if len(grid[r][c].directions) == 0:
            raise RuntimeError(
                "cell ({}, {}) doesn't have a direction.".format(r, c))
        theta = directionToRadians(grid[r][c].directions.pop())
        result.append((x, y, theta))
    return result


def generateSpawnLocations(numberOfLocations):
    try:
        grid = np.load(MAP_CONFIG_FILE_LOCATION)
        cells = [(r, c) for r in range(grid.shape[0])
                 for c in range(grid.shape[1])]
        freeCells = getRandomFreePoints(numberOfLocations, cells, grid)
        points = convertCellsToCoordinates(freeCells, grid)

        with open(GENERATED_SCRIPT_FILE, "w") as f:
            for point in points:
                f.write(ADD_ROBOT_TEMPLATE.format(*point))
    except IOError:
        print(MAP_CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    generateSpawnLocations(parseArgs())
