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
from ..utils.map_information_provider import GRAPH_PICKLED_FILE_LOCATION


TURN_PENALTY = 1.2
HEATMAP_PENALTY = 1


def updateWeights(data):
    heatmap = np.reshape(data.heat_values, (data.rows, data.columns))
    graphWeightsLock.acquire()
    for u, v, d in G.edges(data=True):
        if d.get('weight') is not None:
            timeToMove = (abs(u[0] - v[0]) + abs(u[1] - v[1])) / RobotInfo.getAverageLinearSpeed()
            timeToTurn = abs(u[2] - v[2]) / RobotInfo.getAverageLinearSpeed()
            d['weight'] = timeToMove + timeToTurn * TURN_PENALTY + (1 + HEATMAP_PENALTY * heatmap[v[0]][v[1]])
    graphWeightsLock.release()


def heuristic(currentNode, targetNode):
    timeToMove = (abs(currentNode[0] - targetNode[0]) + abs(currentNode[1] - targetNode[1])) / RobotInfo.getAverageLinearSpeed()
    timeToTurn = abs(currentNode[2] - targetNode[2]) / RobotInfo.getAverageAngularSpeed()
    estimatedPathCost = timeToMove + timeToTurn * TURN_PENALTY
    return estimatedPathCost


def getPath(sourceNode, targetNode):
    nodesInPath = []
    try:
        graphWeightsLock.acquire()
        nodesInPath = nx.astar_path(G, sourceNode, targetNode, heuristic=heuristic)
        graphWeightsLock.release()
        nodesInPath = [State(*node) for node in nodesInPath]
    except nx.NetworkXNoPath:
        print("No path between {} and {}".format(sourceNode, targetNode))
    return nodesInPath


def getPathLength(G, sourceNode, neighbourNode):
    pathLength = None
    try:
        graphWeightsLock.acquire()
        pathLength = nx.astar_path_length(G, sourceNode, neighbourNode, heuristic=heuristic)
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
