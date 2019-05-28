import os
import argparse
import numpy as np
from random import shuffle
from ..map_generation.generate_map_config import Cell
from ..utils import CoordinateSpaceManager

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
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


def generateSpawnLocations(numberOfLocations):
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")
    else:
        grid = mapConfiguration['grid']
        cells = [(r, c) for r in range(grid.shape[0])
                 for c in range(grid.shape[1])]
        freeCells = getRandomFreePoints(numberOfLocations, cells, grid)
        csm = CoordinateSpaceManager()
        points = []
        for cell in freeCells:
            points.append(csm.getWorldCoordinateWithDirection(cell))

        with open(GENERATED_SCRIPT_FILE, "w") as f:
            for point in points:
                f.write(ADD_ROBOT_TEMPLATE.format(*point))


if __name__ == "__main__":
    generateSpawnLocations(parseArgs())
