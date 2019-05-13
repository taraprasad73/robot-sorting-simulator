import argparse
import numpy as np
import copy
from enum import Enum
import os

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ['CATKIN_WORKSPACE']:
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
if not os.path.exists(CATKIN_WORKSPACE + '/src/sorting_robot/data'):
    os.makedirs(CATKIN_WORKSPACE + '/src/sorting_robot/data')
CONFIG_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'


class CellType(Enum):
    PARCEL_BIN = 0
    STREET_STREET_INTERSECTION = 1
    HIGHWAY_HIGHWAY_INTERSECTION = 2
    HIGHWAY_STREET_INTERSECTION = 3
    ONE_WAY_ROAD_ON_STREET = 4
    ONE_WAY_ROAD_ON_HIGHWAY = 5
    RESTRICTED_AREA = 6
    PICKUP_AREA = 7
    BOTTOM_PICKUP_LANES = 8
    CHARGING_AREA = 9
    CHARGING_LANES = 10
    TOP_PICKUP_LANES = 11
    PICKUP_QUEUE_START = 12
    PICKUP_QUEUE_FINISH = 13


class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3


class Turn(Enum):
    LEFT_DOWN = 0
    DOWN_RIGHT = 1
    RIGHT_UP = 2
    UP_LEFT = 3
    DOWN_LEFT = 4
    RIGHT_DOWN = 5
    UP_RIGHT = 6
    LEFT_UP = 7


LEFT_DOWN_DIRECTIONS = set([Direction.LEFT, Direction.DOWN])
DOWN_RIGHT_DIRECTIONS = set([Direction.DOWN, Direction.RIGHT])
RIGHT_UP_DIRECTIONS = set([Direction.RIGHT, Direction.UP])
UP_LEFT_DIRECTIONS = set([Direction.UP, Direction.LEFT])

PROHIBITED_CELL_TYPES = set(
    [CellType.PARCEL_BIN, CellType.RESTRICTED_AREA, CellType.PICKUP_AREA])


# check if cell is inside the grid and does not belong to prohibited cell types
def isCellReachable(grid, cell):
    row, col = cell
    r = grid.shape[0]
    c = grid.shape[1]
    if cell[0] >= 0 and cell[0] < r and cell[1] >= 0 and cell[1] < c:
        if not grid[row][col].cellType in PROHIBITED_CELL_TYPES:
            return True
    return False


class Cell:
    def __init__(self, cellType=None):
        self.row = None
        self.col = None
        self.directions = set()
        self.allowedTurns = set()
        self.roadOrientation = None
        self.cellType = cellType
        self.isOnHighway = False
        self.isOnHorizontalHighway = False
        self.isOnVerticalHighway = False
        self.isOnHorizontalStreet = False
        self.isOnVerticalStreet = False
        self.isObstacle = False
        self.pickupId = 0  # 1 for cells in top pickup region, -1 if cells in bottom pickup region

    def setCellType(self, cellType):
        if self.cellType is None:
            self.cellType = cellType

    def assignCellType(self):
        numDirections = len(self.directions)

        if numDirections == 0:
            self.setCellType(CellType.PARCEL_BIN)
        elif numDirections == 1:
            if self.isOnHighway:
                self.setCellType(CellType.ONE_WAY_ROAD_ON_HIGHWAY)
            else:
                self.setCellType(CellType.ONE_WAY_ROAD_ON_STREET)
        elif numDirections == 2:
            if self.isOnHorizontalHighway and self.isOnVerticalHighway:
                self.setCellType(CellType.HIGHWAY_HIGHWAY_INTERSECTION)
            elif self.isOnHorizontalHighway and self.isOnVerticalStreet:
                self.setCellType(CellType.HIGHWAY_STREET_INTERSECTION)
            elif self.isOnVerticalHighway and self.isOnHorizontalStreet:
                self.setCellType(CellType.HIGHWAY_STREET_INTERSECTION)
            elif self.isOnHorizontalStreet and self.isOnVerticalStreet:
                self.setCellType(CellType.STREET_STREET_INTERSECTION)

    def removeInvalidDirections(self, grid):
        # # special case for the upper pickup lanes which is not handled by the general logic
        if self.cellType == CellType.TOP_PICKUP_LANES:
            if self.directions == LEFT_DOWN_DIRECTIONS:
                self.directions.remove(Direction.LEFT)

        # special case for highways
        if self.isOnHighway:
            if self.roadOrientation == Direction.RIGHT and Direction.DOWN in self.directions:
                self.directions.remove(Direction.DOWN)
            elif self.roadOrientation == Direction.LEFT and Direction.UP in self.directions:
                self.directions.remove(Direction.UP)
            elif self.roadOrientation == Direction.UP and Direction.RIGHT in self.directions:
                self.directions.remove(Direction.RIGHT)
            elif self.roadOrientation == Direction.DOWN and Direction.LEFT in self.directions:
                self.directions.remove(Direction.LEFT)

        # special case for highway-street intersections
        if self.cellType == CellType.HIGHWAY_STREET_INTERSECTION:
            if self.isOnHorizontalHighway:
                if Direction.UP in self.directions:
                    self.directions.remove(Direction.UP)
                if Direction.DOWN in self.directions:
                    self.directions.remove(Direction.DOWN)
            elif self.isOnVerticalHighway:
                if Direction.LEFT in self.directions:
                    self.directions.remove(Direction.LEFT)
                if Direction.RIGHT in self.directions:
                    self.directions.remove(Direction.RIGHT)

        def isMovementFeasible(direction):
            if direction == Direction.LEFT:
                targetCell = [self.row, self.col - 1]
            elif direction == Direction.DOWN:
                targetCell = [self.row + 1, self.col]
            elif direction == Direction.RIGHT:
                targetCell = [self.row, self.col + 1]
            elif direction == Direction.UP:
                targetCell = [self.row - 1, self.col]
            return isCellReachable(grid, targetCell)

        dirList = list(self.directions)
        self.directions.clear()
        for direction in dirList:
            if isMovementFeasible(direction):
                self.directions.add(direction)

    def setAllowedTurns(self, grid):
        if len(self.directions) != 2:
            return

        def isLeftTurnFeasible(turn):
            if turn == Turn.LEFT_DOWN:
                sourceCell = [self.row, self.col + 1]
                targetCell = [self.row + 1, self.col]
            elif turn == Turn.DOWN_RIGHT:
                sourceCell = [self.row - 1, self.col]
                targetCell = [self.row, self.col + 1]
            elif turn == Turn.RIGHT_UP:
                sourceCell = [self.row, self.col - 1]
                targetCell = [self.row - 1, self.col]
            elif turn == Turn.UP_LEFT:
                sourceCell = [self.row + 1, self.col]
                targetCell = [self.row, self.col - 1]
            if isCellReachable(grid, sourceCell) and isCellReachable(grid, targetCell):
                return True
            return False

        def setLeftTurnIfFeasible(turn):
            if isLeftTurnFeasible(turn):
                self.allowedTurns.add(turn)

        # add the left turns
        if self.directions == LEFT_DOWN_DIRECTIONS:
            setLeftTurnIfFeasible(Turn.LEFT_DOWN)
        elif self.directions == DOWN_RIGHT_DIRECTIONS:
            setLeftTurnIfFeasible(Turn.DOWN_RIGHT)
        elif self.directions == RIGHT_UP_DIRECTIONS:
            setLeftTurnIfFeasible(Turn.RIGHT_UP)
        elif self.directions == UP_LEFT_DIRECTIONS:
            setLeftTurnIfFeasible(Turn.UP_LEFT)

        # add the right turns if the cell is on a simple intersection
        if self.cellType == CellType.STREET_STREET_INTERSECTION:
            if self.directions == LEFT_DOWN_DIRECTIONS:
                self.allowedTurns.add(Turn.DOWN_LEFT)
            elif self.directions == DOWN_RIGHT_DIRECTIONS:
                self.allowedTurns.add(Turn.RIGHT_DOWN)
            elif self.directions == RIGHT_UP_DIRECTIONS:
                self.allowedTurns.add(Turn.UP_RIGHT)
            elif self.directions == UP_LEFT_DIRECTIONS:
                self.allowedTurns.add(Turn.LEFT_UP)

    def __str__(self):
        return str(self.__dict__)


def addChargingLanes(n, bottomGrid):
    r = bottomGrid.shape[0]
    c = bottomGrid.shape[1]
    for col in range(n, c, n + 2):
        for row in range(r - 1):
            bottomGrid[row][col].directions.add(Direction.DOWN)
            bottomGrid[row][col].cellType = CellType.CHARGING_LANES
            bottomGrid[row][col + 2].directions.add(Direction.UP)
            bottomGrid[row][col + 2].cellType = CellType.CHARGING_LANES
        bottomGrid[r - 1][col].directions.add(Direction.UP)
        for j in range(col, col + 3):
            bottomGrid[r - 2][j].directions.add(Direction.RIGHT)
            bottomGrid[r - 2][j].cellType = CellType.CHARGING_LANES
    return bottomGrid


def getBottomChargingArea(n, c, chargingRows):
    grid = np.empty(shape=(chargingRows, c), dtype=object)
    for row in range(chargingRows):
        for col in range(c):
            grid[row][col] = Cell(CellType.RESTRICTED_AREA)
    for col in range(n, c, n + 2):
        grid[chargingRows - 1][col].cellType = CellType.CHARGING_AREA
    return grid


def getBottomPickupArea(n, c, pickupRows, pickupColumns):
    grid = np.empty(shape=(pickupRows, c), dtype=object)
    for row in range(pickupRows):
        for col in range(c):
            grid[row][col] = Cell(CellType.RESTRICTED_AREA)

    # start a pickup area with each highway before a block, excluding the last highway
    id = 0
    for start in range(1, c - 1, n + 2):
        id -= 1
        for row in range(0, pickupRows):
            for col in range(start, start + pickupColumns):
                cell = grid[row][col]
                cell.pickupId = id
                if col == start:
                    cell.directions.add(Direction.DOWN)
                    cell.cellType = CellType.BOTTOM_PICKUP_LANES
                elif col == start + pickupColumns - 1:
                    cell.directions.add(Direction.UP)
                    cell.cellType = CellType.BOTTOM_PICKUP_LANES
                else:
                    cell.cellType = CellType.PICKUP_AREA
            if row == pickupRows - 1:
                for col in range(start, start + pickupColumns):
                    grid[row][col].directions.add(Direction.RIGHT)
                    grid[row][col].cellType = CellType.BOTTOM_PICKUP_LANES
        grid[0][start].cellType = CellType.PICKUP_QUEUE_START
        grid[0][start + pickupColumns - 1].cellType = CellType.PICKUP_QUEUE_FINISH
    return grid


def getTopGrid(bottomGrid, n, c):
    grid = copy.deepcopy(bottomGrid)
    grid = np.flip(grid, axis=0)
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            cell = grid[row][col]
            cell.pickupId *= -1
            if cell.cellType == CellType.BOTTOM_PICKUP_LANES:
                cell.cellType = CellType.TOP_PICKUP_LANES
            if Direction.RIGHT in cell.directions:
                cell.directions.remove(Direction.RIGHT)
                cell.directions.add(Direction.LEFT)
            if cell.cellType == CellType.PICKUP_QUEUE_START:
                cell.cellType = CellType.PICKUP_QUEUE_FINISH
            elif cell.cellType == CellType.PICKUP_QUEUE_FINISH:
                cell.cellType = CellType.PICKUP_QUEUE_START
    for col in range(n, c, n + 2):
        grid[0][col].directions.remove(Direction.UP)
        grid[0][col].directions.add(Direction.DOWN)
    return grid


def getCenterGrid(r, c, m, n, p, q):
    grid = np.empty(shape=(r, c), dtype=object)
    for row in range(r):
        for col in range(c):
            grid[row][col] = Cell()

    # assign horizontal arrows
    for row in range(p, r, m + 2):
        # assign the arrows on the current highway
        for cell in grid[row, :]:
            cell.directions.add(Direction.RIGHT)
            cell.isOnHighway = True
            cell.isOnHorizontalHighway = True
            cell.roadOrientation = Direction.RIGHT
        for cell in grid[row + 1, :]:
            cell.directions.add(Direction.LEFT)
            cell.isOnHighway = True
            cell.isOnHorizontalHighway = True
            cell.roadOrientation = Direction.LEFT

        # assign the arrows inside the blocks present after the current highway
        for i in range(row + 3, min(row + m + 1, r), 4):
            for cell in grid[i, :]:
                cell.directions.add(Direction.RIGHT)
                cell.isOnHorizontalStreet = True
        for i in range(row + 5, min(row + m + 1, r), 4):
            for cell in grid[i, :]:
                cell.directions.add(Direction.LEFT)
                cell.isOnHorizontalStreet = True

    # assign vertical arrows
    for col in range(q, c, n + 2):
        # assign the arrows on the current highway
        for cell in grid[:, col]:
            cell.directions.add(Direction.UP)
            cell.isOnHighway = True
            cell.isOnVerticalHighway = True
            cell.roadOrientation = Direction.UP
        for cell in grid[:, col + 1]:
            cell.directions.add(Direction.DOWN)
            cell.isOnHighway = True
            cell.isOnVerticalHighway = True
            cell.roadOrientation = Direction.DOWN

        # assign the arrows inside the blocks present after the current highway
        for j in range(col + 3, min(col + n + 1, c), 4):
            for cell in grid[:, j]:
                cell.directions.add(Direction.UP)
                cell.isOnVerticalStreet = True
        for j in range(col + 5, min(col + n + 1, c), 4):
            for cell in grid[:, j]:
                cell.directions.add(Direction.DOWN)
                cell.isOnVerticalStreet = True

    # assign cell types
    for row in range(r):
        for col in range(c):
            grid[row][col].assignCellType()
    return grid


def checkValidity(r, c, m, n, p, q, pickupRows, pickupColumns, chargingRows, isNotSymmetric):
    # its guranteed that there's atleast two horizontal and vertical highways
    MIN_M = 5
    MIN_N = 5
    MIN_PICKUP_ROWS = 5
    MIN_PICKUP_COLUMNS = 7
    MAX_PICKUP_COLUMNS = n - 2
    MIN_CHARGING_ROWS = 5

    if m < MIN_M:
        parser.error("m should be atleast {}".format(MIN_M))
    if n < MIN_N:
        parser.error("n should be atleast {}".format(MIN_N))

    MIN_R = m + 4
    MIN_C = n + 4

    if isNotSymmetric:
        if r < MIN_R:
            parser.error("r should be atleast {}".format(MIN_R))
        if c < MIN_C:
            parser.error("c should be atleast {}".format(MIN_C))

        MIN_P = 1
        MAX_P = r - (3 + m)

        MIN_Q = 1
        MAX_Q = c - (3 + n)
        if p < MIN_P and p > MAX_P:
            parser.error("p should be between {} and {}".format(MIN_P, MAX_P))
        if q < MIN_Q and q > MAX_Q:
            parser.error("q should be between {} and {}".format(MIN_Q, MAX_Q))
    else:
        if r < MIN_R:
            parser.error("r should be atleast {}".format(MIN_R))
        if c < MIN_C:
            parser.error("c should be atleast {}".format(MIN_C))

        if r % (m + 2) != 2:
            parser.error("r should be of the form (m + 2) * k + 2")
        if c % (n + 2) != 2:
            parser.error("c should be of the form (n + 2) * k + 2")

        if p != 1:
            parser.error("p should be 1 for the grid to be symmetric")
        if q != 1:
            parser.error("q should be 1 for the grid to be symmetric")

    if m % 4 != 1:
        parser.error("m should be of the form 4k + 1")
    if n % 4 != 1:
        parser.error("n should be of the form 4k + 1")

    if pickupColumns < MIN_PICKUP_COLUMNS:
        parser.error(
            "pickup_columns should be atleast {}".format(MIN_PICKUP_COLUMNS))

    if pickupColumns > MAX_PICKUP_COLUMNS:
        parser.error(
            "pickup_columns should be atmost {}".format(MAX_PICKUP_COLUMNS))

    if pickupColumns % 4 != 3:
        parser.error(
            "pickup_columns should be atleast {}".format(MIN_PICKUP_COLUMS))

    if pickupRows < MIN_PICKUP_ROWS:
        parser.error(
            "pickup_rows should be atleast {}".format(MIN_PICKUP_ROWS))

    if chargingRows < MIN_CHARGING_ROWS:
        parser.error(
            "charging_rows should be atleast {}".format(MIN_CHARGING_ROWS))


def parseArgs():
    parser = argparse.ArgumentParser(description="parameters of the bin area")
    parser.add_argument("-r", default=79, type=int,
                        help="total number of rows")
    parser.add_argument("-c", default=79, type=int,
                        help="total number of columns")
    parser.add_argument("-m", default=9, type=int,
                        help="number of rows in a block")
    parser.add_argument("-n", default=9, type=int,
                        help="number of columns in a block")
    parser.add_argument("--pickup-rows", default=9, type=int,
                        help="number of rows in the pickup area")
    parser.add_argument("--pickup-columns", default=7, type=int,
                        help="number of columns in the pickup area")
    parser.add_argument("--charging-rows", default=9, type=int,
                        help="number of rows in the charging area")
    parser.add_argument(
        "-p", default=1, type=int, help="1-indexed row number of first horizontal highway")
    parser.add_argument(
        "-q", default=1, type=int, help="1-indexed column number of first vertical highway")
    parser.add_argument('--is-not-symmetric', default=False,
                        action='store_true', help="whether to keep the grid symmetric or not")
    args = parser.parse_args()
    return args.r, args.c, args.m, args.n, args.p, args.q, args.pickup_rows, args.pickup_columns, args.charging_rows, args.is_not_symmetric


def generateMapConfig():
    r, c, m, n, p, q, pickupRows, pickupColumns, chargingRows, isNotSymmetric = parseArgs()
    if isNotSymmetric:
        print("Not fully implemented yet!")
        exit(1)

    checkValidity(r, c, m, n, p, q, pickupRows,
                  pickupColumns, chargingRows, isNotSymmetric)

    # convert to 0 indexing
    p -= 1
    q -= 1

    centerGrid = getCenterGrid(r, c, m, n, p, q)
    bottomPickupArea = getBottomPickupArea(n, c, pickupRows, pickupColumns)
    bottomChargingArea = getBottomChargingArea(n, c, chargingRows)
    bottomGrid = np.concatenate((bottomPickupArea, bottomChargingArea), axis=0)
    bottomGrid = addChargingLanes(n, bottomGrid)
    topGrid = getTopGrid(bottomGrid, n, c)
    grid = np.concatenate(
        (topGrid, centerGrid, bottomGrid), axis=0)

    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            grid[row][col].row = row
            grid[row][col].col = col

    # there are invalid directions, but they make turn assignment easier
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            grid[row][col].setAllowedTurns(grid)

    # remove the invalid directions
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            grid[row][col].removeInvalidDirections(grid)

    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row][col].cellType in PROHIBITED_CELL_TYPES:
                grid[row][col].isObstacle = True

    np.save(CONFIG_FILE_SAVE_LOCATION, grid)

    '''
    pickups = {
        1: {'start': (17, 7), 'finish': (17, 1)},
        2: {'start': (17, 18), 'finish': (17, 12)},
        3: {'start': (17, 29), 'finish': (17, 23)},
        4: {'start': (17, 40), 'finish': (17, 34)},
        5: {'start': (17, 51), 'finish': (17, 45)},
        6: {'start': (17, 62), 'finish': (17, 56)},
        7: {'start': (17, 73), 'finish': (17, 67)},
        -1: {'start': (97, 1), 'finish': (97, 7)},
        -2: {'start': (97, 12), 'finish': (97, 18)},
        -3: {'start': (97, 23), 'finish': (97, 29)},
        -4: {'start': (97, 34), 'finish': (97, 40)},
        -5: {'start': (97, 45), 'finish': (97, 51)},
        -6: {'start': (97, 56), 'finish': (97, 62)},
        -7: {'start': (97, 67), 'finish': (97, 73)},
    }
    '''

    pickups = {}
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row][col].cellType == CellType.PICKUP_QUEUE_START:
                if pickups.get(grid[row][col].pickupId) is None:
                    pickups[grid[row][col].pickupId] = {
                        'start': (row, col), 'finish': None}
                else:
                    pickups[grid[row][col].pickupId]['start'] = (row, col)
            elif grid[row][col].cellType == CellType.PICKUP_QUEUE_FINISH:
                if pickups.get(grid[row][col].pickupId) is None:
                    pickups[grid[row][col].pickupId] = {
                        'start': None, 'finish': (row, col)}
                else:
                    pickups[grid[row][col].pickupId]['finish'] = (row, col)
                pickups[grid[row][col].pickupId]['finish'] = (row, col)

    mapConfiguration = {
        'grid': grid,
        'pickups': pickups,
        'pickup_queue_size': pickupRows * 2 + pickupColumns - 2,
        'num_rows': grid.shape[0],
        'num_columns': grid.shape[1],
        'cell_length_in_meters': 0.5,
    }
    np.save(CONFIG_FILE_SAVE_LOCATION, np.array(mapConfiguration))


if __name__ == "__main__":
    generateMapConfig()
