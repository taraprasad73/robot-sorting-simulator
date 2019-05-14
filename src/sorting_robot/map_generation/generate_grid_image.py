'''
TODO add color coded legend, pick a different color for each cell type
'''

import numpy as np
import os
if os.environ.get('CIRCLECI'):
    import matplotlib
    matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.table import Table
from matplotlib.lines import Line2D
from enum import Enum
from generate_map_config import Cell, Direction, Turn, CellType

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
MAP_IMAGE_FILE_SVG_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map.svg'
MAP_IMAGE_FILE_PNG_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map.png'
MAP_LEGEND_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_legend.png'

LEFT_ARROW = u"\u2190"
RIGHT_ARROW = u"\u2192"
UP_ARROW = u"\u2191"
DOWN_ARROW = u"\u2193"

LEFT_DOWN_ARROW = u"\u2B10"
DOWN_RIGHT_ARROW = u"\u21B3"
RIGHT_UP_ARROW = u"\u2B0F"
UP_LEFT_ARROW = u"\u21B0"

ONE_WAY_ROAD_ON_HIGHWAY_COLOR = 'gold'
ONE_WAY_ROAD_ON_STREET_COLOR = 'white'
STREET_STREET_INTERSECTION_COLOR = 'lightgray'
HIGHWAY_HIGHWAY_INTERSECTION_COLOR = 'violet'
HIGHWAY_STREET_FORK_COLOR = 'greenyellow'
HIGHWAY_STREET_INTERSECTION_COLOR = 'aquamarine'
BIN_COLOR = 'black'
PICKUP_LANE_COLOR = 'yellow'
PICKUP_AREA_COLOR = 'lightsteelblue'
RESTRICTED_AREA_COLOR = 'gray'
CHARGING_AREA_COLOR = 'brown'
CHARGING_LANE_COLOR = 'pink'
PICKUP_QUEUE_START_COLOR = 'green'
PICKUP_QUEUE_FINISH_COLOR = 'red'


def setDirection(cell):
    content = ""
    if len(cell.directions) > 0:
        direction = list(cell.directions)[0]
        if(direction == Direction.LEFT):
            content = LEFT_ARROW
        elif(direction == Direction.RIGHT):
            content = RIGHT_ARROW
        elif(direction == Direction.UP):
            content = UP_ARROW
        elif(direction == Direction.DOWN):
            content = DOWN_ARROW
    return content


def getTurnContent(turn):
    if turn == Turn.UP_LEFT:
        content = UP_LEFT_ARROW
    elif turn == Turn.DOWN_RIGHT:
        content = DOWN_RIGHT_ARROW
    elif turn == Turn.RIGHT_UP:
        content = RIGHT_UP_ARROW
    elif turn == Turn.LEFT_DOWN:
        content = LEFT_DOWN_ARROW
    return content


def getColor(cellType):
    color = "white"
    if cellType == CellType.PARCEL_BIN:
        color = BIN_COLOR
    elif cellType == CellType.ONE_WAY_ROAD_ON_HIGHWAY:
        color = ONE_WAY_ROAD_ON_HIGHWAY_COLOR
    elif cellType == CellType.ONE_WAY_ROAD_ON_STREET:
        color = ONE_WAY_ROAD_ON_STREET_COLOR
    elif cellType == CellType.STREET_STREET_INTERSECTION:
        color = STREET_STREET_INTERSECTION_COLOR
    elif cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION:
        color = HIGHWAY_HIGHWAY_INTERSECTION_COLOR
    elif cellType == CellType.HIGHWAY_STREET_INTERSECTION:
        color = HIGHWAY_STREET_INTERSECTION_COLOR
    elif cellType == CellType.HIGHWAY_STREET_FORK:
        color = HIGHWAY_STREET_FORK_COLOR
    elif cellType == CellType.RESTRICTED_AREA:
        color = RESTRICTED_AREA_COLOR
    elif cellType == CellType.CHARGING_AREA:
        color = CHARGING_AREA_COLOR
    elif cellType == CellType.PICKUP_AREA:
        color = PICKUP_AREA_COLOR
    elif cellType == CellType.BOTTOM_PICKUP_LANES or cellType == CellType.TOP_PICKUP_LANES:
        color = PICKUP_LANE_COLOR
    elif cellType == CellType.CHARGING_LANES:
        color = CHARGING_LANE_COLOR
    elif cellType == CellType.PICKUP_QUEUE_START:
        color = PICKUP_QUEUE_START_COLOR
    elif cellType == CellType.PICKUP_QUEUE_FINISH:
        color = PICKUP_QUEUE_FINISH_COLOR
    return color


def createLegend(colorDict):
    fig, ax = plt.subplots()
    ax.set_axis_off()
    coloredLines = [Line2D([0], [0], color=color, lw=8) for color in colorDict.values()]
    cellTypes = [cellType.name for cellType in colorDict.keys()]
    ax.legend(coloredLines, cellTypes)
    plt.savefig(MAP_LEGEND_FILE_SAVE_LOCATION, format='png', dpi=1200)


def createGridImage(colorDict):
    mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
    data = mapConfiguration['grid']
    rows, cols = data.shape[0], data.shape[1]
    width, height = 1.5 / cols, 1.5 / rows
    fig, ax = plt.subplots()
    ax.set_axis_off()
    ax.set_aspect('equal', 'box')
    grid = Table(ax, bbox=[0, 0, 1, 1])
    for i in range(0, rows):
        for j in range(0, cols):
            cell = data[i][j]
            color = colorDict[cell.cellType]
            content = ""
            if not cell.isObstacle:
                if len(cell.allowedTurns) > 1:
                    content = '+'
                elif len(cell.allowedTurns) == 1:
                    turn = list(cell.allowedTurns)[0]
                    content = getTurnContent(turn)
                else:
                    content = setDirection(cell)
            grid.add_cell(i, j, width, height, text=content,
                          loc='center', facecolor=color)

    # this thing might also be done at the table level
    for key, cell in grid.get_celld().items():
        cell.set_linewidth(0)
        cell.set_fontsize(4)

    ax.add_table(grid)

    plt.tight_layout(rect=[0.01, 0.01, 0.99, 0.99])

    plt.savefig(MAP_IMAGE_FILE_PNG_SAVE_LOCATION, format='png', dpi=1200)
    plt.savefig(MAP_IMAGE_FILE_SVG_SAVE_LOCATION, format='svg', dpi=1200)


def generateGridImage():
    colorDict = {}
    for cellType in CellType:
        colorDict[cellType] = getColor(cellType)
    try:
        createGridImage(colorDict)
        createLegend(colorDict)
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    generateGridImage()
