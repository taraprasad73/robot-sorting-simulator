import numpy as np
from ..map_generation.generate_map_config import Cell, Direction, Turn, CellType
from random import shuffle
import math
import argparse
import os

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ['CATKIN_WORKSPACE']:
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
GENERATED_SCRIPT_FILE = CATKIN_WORKSPACE + '/src/sorting_robot/data/spawn_robots.sh'

ADD_ROBOT_TEMPLATE = 'rosrun stdr_robot robot_handler add $HOME/catkin_ws/src/sorting_robot/stdr_data/robots/pandora_robot.yaml {} {} {}\n'


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


def convertCellsToCoordinates(freeCells, grid, cellLength):
    result = []
    RANGE_Y = grid.shape[0] * cellLength
    for (r, c) in freeCells:
        x = (c + 0.5) * cellLength
        y = (r + 0.5) * cellLength
        y = RANGE_Y - y
        if len(grid[r][c].directions) == 0:
            raise RuntimeError(
                "cell ({}, {}) doesn't have a direction.".format(r, c))
        theta = directionToRadians(grid[r][c].directions.pop())
        result.append((x, y, theta))
    return result


def generateSpawnLocations(numberOfLocations):
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
        grid = mapConfiguration['grid']
        cellLength = mapConfiguration['cell_length_in_meters']
        cells = [(r, c) for r in range(grid.shape[0])
                 for c in range(grid.shape[1])]
        freeCells = getRandomFreePoints(numberOfLocations, cells, grid)
        points = convertCellsToCoordinates(freeCells, grid, cellLength)

        with open(GENERATED_SCRIPT_FILE, "w") as f:
            for point in points:
                f.write(ADD_ROBOT_TEMPLATE.format(*point))
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    generateSpawnLocations(parseArgs())
