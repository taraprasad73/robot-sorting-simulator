import os
import numpy as np
import networkx as nx
if os.environ.get('CIRCLECI'):
    import matplotlib
    matplotlib.use('agg')
import matplotlib.pyplot as plt
from generate_map_config import Cell, Direction, Turn, CellType


HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
GRAPH_PICKLED_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/graph.gpickle'
GRAPH_IMAGE_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/graph.svg'

TURN_COST = 50
MOVE_COST = 20

CELL_LENGTH = 200


def addEdges(grid, graph):
    for row in range(0, grid.shape[0]):
        for col in range(0, grid.shape[1]):
            cell = grid[row][col]
            if cell.isObstacle:
                continue
            for direction in cell.directions:
                if(direction == Direction.RIGHT):
                    graph.add_edge(
                        (row, col, 0), (row, col + 1, 0), weight=MOVE_COST)
                if(direction == Direction.UP):
                    graph.add_edge(
                        (row, col, 90), (row - 1, col, 90), weight=MOVE_COST)
                if(direction == Direction.LEFT):
                    graph.add_edge((row, col, 180),
                                   (row, col - 1, 180), weight=MOVE_COST)
                if(direction == Direction.DOWN):
                    graph.add_edge((row, col, 270),
                                   (row + 1, col, 270), weight=MOVE_COST)

            for turn in cell.allowedTurns:
                if(turn == Turn.UP_LEFT):
                    graph.add_edge(
                        (row, col, 90), (row, col, 180), weight=TURN_COST)
                elif(turn == Turn.DOWN_RIGHT):
                    graph.add_edge((row, col, 270),
                                   (row, col, 0), weight=TURN_COST)
                elif(turn == Turn.RIGHT_UP):
                    graph.add_edge(
                        (row, col, 0), (row, col, 90), weight=TURN_COST)
                elif(turn == Turn.LEFT_DOWN):
                    graph.add_edge((row, col, 180),
                                   (row, col, 270), weight=TURN_COST)
                elif(turn == Turn.LEFT_UP):
                    graph.add_edge((row, col, 180),
                                   (row, col, 90), weight=TURN_COST)
                elif(turn == Turn.RIGHT_DOWN):
                    graph.add_edge(
                        (row, col, 0), (row, col, 270), weight=TURN_COST)
                elif(turn == Turn.UP_RIGHT):
                    graph.add_edge(
                        (row, col, 90), (row, col, 0), weight=TURN_COST)
                elif(turn == Turn.DOWN_LEFT):
                    graph.add_edge((row, col, 270),
                                   (row, col, 180), weight=TURN_COST)


def generateNetworkxGraph():
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
        grid = mapConfiguration['grid']
        G = nx.DiGraph()
        addEdges(grid, G)

        pos = {}
        for node in G.nodes():
            center = (node[1] * 3 * CELL_LENGTH, -node[0] * 3 * CELL_LENGTH)
            if node[2] == 0:
                pos[node] = (center[0] + CELL_LENGTH, center[1])
            elif node[2] == 90:
                pos[node] = (center[0], center[1] + CELL_LENGTH)
            elif node[2] == 180:
                pos[node] = (center[0] - CELL_LENGTH, center[1])
            elif node[2] == 270:
                pos[node] = (center[0], center[1] - CELL_LENGTH)

        nx.draw(G, pos=pos, arrowsize=2, node_size=0.1)
        plt.savefig(GRAPH_IMAGE_FILE_SAVE_LOCATION, dpi=4800)
        nx.write_gpickle(G, GRAPH_PICKLED_FILE_SAVE_LOCATION)
    except:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    generateNetworkxGraph()
