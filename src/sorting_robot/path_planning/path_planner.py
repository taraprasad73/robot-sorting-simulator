import os
if os.environ.get('CIRCLECI'):
    import matplotlib
    matplotlib.use('agg')
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rospy
from sorting_robot.msg import State
from sorting_robot.srv import Path, PathToBin
from ..utils import RobotInfo

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
ANNOTATED_GRAPH_IMAGE_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/annotated_graph.svg'
GRAPH_PICKLED_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/graph.gpickle'

TURN_PENALTY = 1.2
HEATMAP_PENALTY = 1


def drawPath(self, nodesInPath):
    edgelist = [(nodesInPath[i], nodesInPath[i + 1]) for i in range(len(nodesInPath) - 1)]
    CELL_LENGTH = 200
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
    nx.draw(G, pos=pos, arrowsize=2, node_size=0.1, edgecolor='green')
    nx.draw_networkx_edges(G, pos=pos, edgelist=edgelist, arrowsize=6, color='red')
    plt.savefig(ANNOTATED_GRAPH_IMAGE_FILE_SAVE_LOCATION, dpi=4800)


def updateWeights(data):
    heatmap = np.reshape(data.heat_values, (data.rows, data.columns))
    for u, v, d in G.edges(data=True):
        if d.get('weight') is not None:
            timeToMove = (abs(u[0] - v[0]) + abs(u[1] - v[1])) / RobotInfo.getAverageLinearSpeed()
            timeToTurn = abs(u[2] - v[2]) / RobotInfo.getAverageLinearSpeed()
            d['weight'] = timeToMove + timeToTurn * TURN_PENALTY + (1 + HEATMAP_PENALTY * heatmap[v[0]][v[1]])


def heuristic(self, currentNode, targetNode):
    timeToMove = (abs(currentNode[0] - targetNode[0]) + abs(currentNode[1] - targetNode[1])) / RobotInfo.getAverageLinearSpeed()
    timeToTurn = abs(currentNode[2] - targetNode[2]) / RobotInfo.getAverageLinearSpeed()
    estimatedPathCost = timeToMove + timeToTurn * TURN_PENALTY
    return estimatedPathCost


def getPathResponse(sourceNode, targetNode):
    nodesInPath = []
    if sourceNode in G and targetNode in G:
        try:
            nodesInPath = nx.astar_path(G, sourceNode, targetNode)
            nodesInPath = [State(*node) for node in nodesInPath]
        except nx.NetworkXNoPath:
            print("No path between {} and {}".format(sourceNode, targetNode))
    else:
        print("Either {} or {} doesn't exist as a node in graph".format(sourceNode, targetNode))
    print(nodesInPath)
    return PathResponse(path=nodesInPath)


def handlePathToBinRequest(req):
    source = req.source
    bin_position = req.bin_position
    sourceNode = (source.row, source.col, source.direction)
    binNode = (bin_position.row, bin_position.col)

    if sourceNode in G and binNode in G:
        pathLengths = {}
        for neighbourNode in G[binNode]:
            try:
                pathLength = nx.astar_path_length(G, sourceNode, neighbourNode)
                pathLengths[neighbourNode] = pathLength
            except nx.NetworkXNoPath:
                print("No path between {} and {}".format(sourceNode, neighbourNode))
        if len(pathLengths) > 0:
            nearestNode = min(pathLengths, key=pathLengths.get)
            print("Calculating path...")
            return getPathResponse(sourceNode, nearestNode)
        else:
            print("No path to reach the bin {} from {}".format(binNode, sourceNode))
    else:
        print("Either {} or {} doesn't exist as a node in graph".format(sourceNode, binNode))
    return None


def handlePathRequest(req):
    source = req.source
    destination = req.destination
    sourceNode = (source.row, source.col, source.direction)
    targetNode = (destination.row, destination.col, destination.direction)
    return getPathResponse(sourceNode, targetNode)


def pathPlanner():
    try:
        global G
        G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
    except IOError:
        print(GRAPH_PICKLED_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_networkx_graph")
    else:
        try:
            mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
        except IOError:
            print(CONFIG_FILE_LOCATION +
                  " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")
        else:
            rospy.init_node('path_planning_server')
            rospy.Subscriber('heat_map', HeatMap, updateWeights)
            pathService = rospy.Service('path', Path, handlePathRequest)
            pathToBinService = rospy.Service('path_to_bin', PathToBin, handlePathToBinRequest)
            print('path planning server is running...')
            rospy.spin()


if __name__ == "__main__":
    pathPlanner()
