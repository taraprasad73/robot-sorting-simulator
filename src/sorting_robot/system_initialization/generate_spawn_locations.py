import os
import argparse
import subprocess
import time
import numpy as np
from random import shuffle
from ..map_generation.generate_map_config import Cell
from ..utils import CoordinateSpaceManager
from ..utils.map_information_provider import CONFIG_FILE_LOCATION, SPAWN_LOCATIONS_SCRIPT_FILE
from ..utils.robot_info import ROBOT_CONFIGURATION_FILE_LOCATION

ADD_ROBOT_TEMPLATE = 'rosrun stdr_robot robot_handler add {} {} {} {}\n'
WAIT_TIME_BETWEEN_CALLS = 3


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
        # freeCells = getRandomFreePoints(numberOfLocations, cells, grid)
        freeCells = [(9, 3), (11, 3)]
        csm = CoordinateSpaceManager()
        points = []
        for cell in freeCells:
            points.append(csm.getWorldCoordinateWithDirection(cell))

        commandsList = []
        with open(SPAWN_LOCATIONS_SCRIPT_FILE, "w") as f:
            for point in points:
                bashCommand = ADD_ROBOT_TEMPLATE.format(ROBOT_CONFIGURATION_FILE_LOCATION, *point)
                f.write(bashCommand)
                commandsList.append(bashCommand)


def executeOneByOne():
    with open(SPAWN_LOCATIONS_SCRIPT_FILE, "r") as f:
        commandsList = f.read().splitlines()
        count = 0
        for bashCommand in commandsList:
            process = subprocess.Popen(bashCommand, stdout=subprocess.PIPE, shell=True)
            output, error = process.communicate()
            print("Shell Output: " + output)
            if error is not None:
                print("Shell Error: " + error)
            if count != len(commandsList) - 1:
                print('waiting {} seconds for simulator to process the current request...'.format(WAIT_TIME_BETWEEN_CALLS))
                time.sleep(WAIT_TIME_BETWEEN_CALLS)
            count += 1


def executeAllAtOnce():
    launchScriptCommand = 'bash {}'.format(SPAWN_LOCATIONS_SCRIPT_FILE)
    process = subprocess.Popen(launchScriptCommand, stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()
    print("Shell Output:\n" + output)
    if error is not None:
        print("Shell Error: " + error)

