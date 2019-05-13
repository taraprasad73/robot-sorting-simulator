import numpy as np
import matplotlib.pyplot as plt
from matplotlib.table import Table
from enum import Enum
from generate_map_config import Cell, Direction, Turn, CellType
import os

HOME_DIR = os.environ['HOME']
MAP_CONFIG_FILE_LOCATION = HOME_DIR + '/catkin_ws/src/sorting_robot/data/map_configuration.npy'
MAP_IMAGE_FILE_SVG_SAVE_LOCATION = HOME_DIR + '/catkin_ws/src/sorting_robot/data/map.svg'
MAP_IMAGE_FILE_PNG_SAVE_LOCATION = HOME_DIR + '/catkin_ws/src/sorting_robot/data/map.png'

LEFT_ARROW = u"\u2190"
RIGHT_ARROW = u"\u2192"
UP_ARROW = u"\u2191"
DOWN_ARROW = u"\u2193"

LEFT_DOWN_ARROW = u"\u2B10"
DOWN_RIGHT_ARROW = u"\u21B3"
RIGHT_UP_ARROW = u"\u2B0F"
UP_LEFT_ARROW = u"\u21B0"


ONE_WAY_ROAD_ON_HIGHWAY_COLOR = 'orange'
ONE_WAY_ROAD_ON_STREET_COLOR = 'white'
STREET_STREET_INTERSECTION_COLOR = 'white'
HIGHWAY_HIGHWAY_INTERSECTION_COLOR = 'green'
HIGHWAY_STREET_INTERSECTION_COLOR = 'pink'
BIN_COLOR = 'black'
PICKUP_LANE_COLOR = 'yellow'
PICKUP_AREA_COLOR = 'blue'
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


def getColor(cell):
    color = "white"
    if cell.cellType == CellType.PARCEL_BIN:
        color = BIN_COLOR
    elif cell.cellType == CellType.ONE_WAY_ROAD_ON_HIGHWAY:
        color = ONE_WAY_ROAD_ON_HIGHWAY_COLOR
    elif cell.cellType == CellType.ONE_WAY_ROAD_ON_STREET:
        color = ONE_WAY_ROAD_ON_STREET_COLOR
    elif cell.cellType == CellType.STREET_STREET_INTERSECTION:
        color = STREET_STREET_INTERSECTION_COLOR
    elif cell.cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION:
        color = HIGHWAY_HIGHWAY_INTERSECTION_COLOR
    elif cell.cellType == CellType.HIGHWAY_STREET_INTERSECTION:
        color = HIGHWAY_STREET_INTERSECTION_COLOR
    elif cell.cellType == CellType.RESTRICTED_AREA:
        color = RESTRICTED_AREA_COLOR
    elif cell.cellType == CellType.CHARGING_AREA:
        color = CHARGING_AREA_COLOR
    elif cell.cellType == CellType.PICKUP_AREA:
        color = PICKUP_AREA_COLOR
    elif cell.cellType == CellType.BOTTOM_PICKUP_LANES or cell.cellType == CellType.TOP_PICKUP_LANES:
        color = PICKUP_LANE_COLOR
    elif cell.cellType == CellType.CHARGING_LANES:
        color = CHARGING_LANE_COLOR
    elif cell.cellType == CellType.PICKUP_QUEUE_START:
        color = PICKUP_QUEUE_START_COLOR
    elif cell.cellType == CellType.PICKUP_QUEUE_FINISH:
        color = PICKUP_QUEUE_FINISH_COLOR
    return color


def generateGridImage():
    try:
        mapConfiguration = np.load(MAP_CONFIG_FILE_LOCATION).item()
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
                color = getColor(cell)
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
    except IOError:
        print(MAP_CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    generateGridImage()
