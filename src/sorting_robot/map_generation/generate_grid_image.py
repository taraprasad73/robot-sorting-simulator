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

LABEL_COLOR = 'white'
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


def createLegend(colorDict, saveFig):
    fig, ax = plt.subplots()
    ax.set_axis_off()
    coloredLines = [Line2D([0], [0], color=color, lw=8) for color in colorDict.values()]
    cellTypes = [cellType.name for cellType in colorDict.keys()]
    ax.legend(coloredLines, cellTypes)
    if saveFig:
        plt.savefig(MAP_LEGEND_FILE_SAVE_LOCATION, format='png', dpi=1200)


def createGridImage(data, colorDict, saveFig, imageFormat):
    rows, cols = data.shape[0] + 1, data.shape[1] + 1  # +1 for the labels
    # cellWidth = 1/rows limits the max font size to a small value, hence used 2/rows, but the issue is still there
    cellWidth, cellHeight = 2.0 / rows, 2.0 / rows
    fig, ax = plt.subplots()
    ax.set_axis_off()
    ax.set_aspect('equal', 'box')

    colLength = cols / float(rows)
    colStart = (1 - colLength) / 2.0
    grid = Table(ax, bbox=[colStart, 0, colStart + colLength, 1])

    for i in range(rows):
        for j in range(cols):
            content = ""
            cellColor = LABEL_COLOR
            if i == 0 and j == 0:
                pass
            elif i == 0:
                content = str(j - 1)
            elif j == 0:
                content = str(i - 1)
            else:
                cell = data[i - 1][j - 1]
                cellColor = colorDict[cell.cellType]

                if not cell.isObstacle:
                    if len(cell.allowedTurns) > 1:
                        content = '+'
                    elif len(cell.allowedTurns) == 1:
                        turn = list(cell.allowedTurns)[0]
                        content = getTurnContent(turn)
                    else:
                        content = setDirection(cell)
            grid.add_cell(i, j, cellWidth, cellHeight, text=content,
                          loc='center', facecolor=cellColor)

    # grid.get_celld() returns a dict
    for (row, col), cell in grid.get_celld().items():
        if row == 0 or col == 0 or data[row - 1][col - 1].cellType == CellType.RESTRICTED_AREA:
            cell.set_linewidth(0.15)
        else:
            cell.set_linewidth(0)
        if row == 0 or col == 0:
            cell.set_fontsize(3.5)
        else:
            cell.set_fontsize(5)

    ax.add_table(grid)

    if saveFig:
        plt.tight_layout(rect=[0.01, 0.01, 0.99, 0.99])
        if imageFormat == 'png':
            plt.savefig(MAP_IMAGE_FILE_PNG_SAVE_LOCATION, format=imageFormat, dpi=1200)
        elif imageFormat == 'svg':
            plt.savefig(MAP_IMAGE_FILE_SVG_SAVE_LOCATION, format=imageFormat, dpi=1200)


def generateGridImage(saveFig=False, imageFormat='png'):
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")
    else:
        colorDict = {}
        for cellType in CellType:
            colorDict[cellType] = getColor(cellType)
        data = mapConfiguration['grid']
        createGridImage(data, colorDict, saveFig, imageFormat)
        createLegend(colorDict, saveFig)


if __name__ == "__main__":
    generateGridImage(saveFig=True, imageFormat='png')
    generateGridImage(saveFig=True, imageFormat='svg')
