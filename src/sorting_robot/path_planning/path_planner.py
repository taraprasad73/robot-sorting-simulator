import os
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sorting_robot.msg import State
from sorting_robot.srv import *

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
ANNOTATED_GRAPH_IMAGE_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/annotated_graph.svg'
GRAPH_PICKLED_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/graph.gpickle'


def handlePathToBinRequest(req):
    pass


def handlePathRequest(req):
    source = req.source
    destination = req.destination
    nodes = pathPlanner.astar((source.row, source.col, source.direction), (destination.row, destination.col, destination.direction))
    nodes = [State(*node) for node in nodes]
    return PathResponse(path=nodes)


class PathPlanner:
    def __init__(self, G):
        self.G = G
        rospy.init_node('path_planning_server')
        pathService = rospy.Service('path', Path, handlePathRequest)
        pathToBinService = rospy.Service('path_to_bin', PathToBin, handlePathToBinRequest)

    def heuristic(self, currentNode, targetNode):
        pass

    def astar(self, sourceNode, targetNode):
        if sourceNode in self.G and targetNode in self.G:
            try:
                nodesInPath = nx.astar_path(self.G, sourceNode, targetNode)
                return nodesInPath
            except nx.NetworkXNoPath:
                return None
        else:
            print("Node doesn't exist")
            return None

    def drawPath(self, G, nodesInPath):
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


def pathPlanner():
    try:
        G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
        global pathPlanner
        pathPlanner = PathPlanner(G)
        # edgelist = astar(G, (27, 48, 180), (60, 49, 180))
        rospy.spin()
    except IOError:
        print(GRAPH_PICKLED_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_networkx_graph")


if __name__ == "__main__":
    pathPlanner()
