import os
if os.environ.get('CIRCLECI'):
    import matplotlib
    matplotlib.use('agg')
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rospy
from threading import Lock
from sorting_robot.msg import State, HeatMap
from sorting_robot.srv import Path, PathToBin, PathResponse, PathToBinResponse
from ..utils import RobotInfo

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
# CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
ANNOTATED_GRAPH_IMAGE_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/annotated_graph.svg'
GRAPH_PICKLED_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/graph.gpickle'

TURN_PENALTY = 1.2
HEATMAP_PENALTY = 1


'''
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
    plt.savefig(ANNOTATED_GRAPH_IMAGE_FILE_SAVE_LOCATION, dpi=1200)
'''


def updateWeights(data):
    heatmap = np.reshape(data.heat_values, (data.rows, data.columns))
    graphWeightsLock.acquire()
    for u, v, d in G.edges(data=True):
        if d.get('weight') is not None:
            timeToMove = (abs(u[0] - v[0]) + abs(u[1] - v[1])) / RobotInfo.getAverageLinearSpeed()
            timeToTurn = abs(u[2] - v[2]) / RobotInfo.getAverageLinearSpeed()
            d['weight'] = timeToMove + timeToTurn * TURN_PENALTY + (1 + HEATMAP_PENALTY * heatmap[v[0]][v[1]])
    graphWeightsLock.release()


def heuristic(self, currentNode, targetNode):
    timeToMove = (abs(currentNode[0] - targetNode[0]) + abs(currentNode[1] - targetNode[1])) / RobotInfo.getAverageLinearSpeed()
    timeToTurn = abs(currentNode[2] - targetNode[2]) / RobotInfo.getAverageLinearSpeed()
    estimatedPathCost = timeToMove + timeToTurn * TURN_PENALTY
    return estimatedPathCost


def getPath(sourceNode, targetNode):
    nodesInPath = []
    try:
        graphWeightsLock.acquire()
        nodesInPath = nx.astar_path(G, sourceNode, targetNode)
        graphWeightsLock.release()
        nodesInPath = [State(*node) for node in nodesInPath]
    except nx.NetworkXNoPath:
        print("No path between {} and {}".format(sourceNode, targetNode))
    return nodesInPath


def getPathLength(G, sourceNode, neighbourNode):
    pathLength = None
    try:
        graphWeightsLock.acquire()
        pathLength = nx.astar_path_length(G, sourceNode, neighbourNode)
        graphWeightsLock.release()
    except nx.NetworkXNoPath:
        print("No path between {} and {}".format(sourceNode, neighbourNode))
    return pathLength


def handlePathToBinRequest(req):
    source = req.source
    bin_position = req.bin_position
    sourceNode = (source.row, source.col, source.direction)
    binNode = (bin_position.row, bin_position.col)

    if sourceNode not in G or binNode not in G:
        print("Either {} or {} doesn't exist as a node in graph".format(sourceNode, binNode))
    else:
        pathLengths = {}
        for neighbourNode in G[binNode]:
            pathLength = getPathLength(G, sourceNode, neighbourNode)
            if pathLength is not None:
                pathLengths[neighbourNode] = pathLength
        if len(pathLengths) > 0:
            nearestNode = min(pathLengths, key=pathLengths.get)
            return PathToBinResponse(path=getPath(sourceNode, nearestNode))
        else:
            print("No path to reach the bin {} from {}".format(binNode, sourceNode))
    return None


def handlePathRequest(req):
    source = req.source
    destination = req.destination
    sourceNode = (source.row, source.col, source.direction)
    targetNode = (destination.row, destination.col, destination.direction)
    if sourceNode not in G or targetNode not in G:
        print("Either {} or {} doesn't exist as a node in graph".format(sourceNode, targetNode))
        return None
    return PathResponse(path=getPath(sourceNode, targetNode))


def pathPlanner():
    try:
        global G, graphWeightsLock
        G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
        graphWeightsLock = Lock()
    except IOError:
        print(GRAPH_PICKLED_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_networkx_graph")
    else:
        rospy.init_node('path_planning_server')
        rospy.Subscriber('heat_map', HeatMap, updateWeights)
        pathService = rospy.Service('path', Path, handlePathRequest)
        pathToBinService = rospy.Service('path_to_bin', PathToBin, handlePathToBinRequest)
        print('path planning server is running...')
        rospy.spin()


if __name__ == "__main__":
    pathPlanner()
