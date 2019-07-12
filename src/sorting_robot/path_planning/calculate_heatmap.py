import argparse
import rospy
import re
import logging
import threading
import datetime
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sorting_robot.msg import HeatMap, OccupancyMap
from ..utils import CoordinateSpaceManager, MapInformationProvider, RobotInfo

HEATMAP_PUBLISH_RATE = 10
TOPIC_SEARCH_INTERVAL = 0.5
# TODO heatmap evaporates rapidly, needs fix
ETA = 0.9999999  # per second


class Heatmap:
    def __init__(self, numRows, numColumns):
        rospy.init_node('heatmap', anonymous=False, log_level=rospy.INFO)
        self.csm = CoordinateSpaceManager()
        self.gridShape = (numRows, numColumns)
        self.heatmapPublisher = rospy.Publisher('/heat_map', HeatMap, queue_size=10)
        self.occupancyPublisher = rospy.Publisher('/occupancy_map', OccupancyMap, queue_size=HEATMAP_PUBLISH_RATE)
        self.activeRobots = {}
        self.lastHeatmapCalaculatedTime = datetime.datetime.now()
        self.previousMap = np.zeros((numRows, numColumns))
        rospy.Timer(rospy.Duration(TOPIC_SEARCH_INTERVAL), self.findTopics)
        self.activeRobotsLock = threading.Lock()

    def odomCallback(self, data, robotName):
        self.activeRobotsLock.acquire()
        self.activeRobots[robotName]['pose'] = data.pose.pose
        self.activeRobotsLock.release()

    def findTopics(self, data):
        topics = rospy.get_published_topics()
        newRobots = set()
        for topic in topics:
            topic_name, topic_type = topic
            match = re.search("/.*/odom", topic_name)
            if(match):
                check = re.search("/.*/", topic_name)
                robotName = check.group()[1:len(check.group()) - 1]
                if self.activeRobots.get(robotName) is None:
                    self.activeRobots[robotName] = {
                        'subscriber': rospy.Subscriber(topic_name, Odometry, self.odomCallback, robotName)
                    }
                newRobots.add(robotName)

        retiredRobots = []
        for robotName in self.activeRobots.keys():
            if robotName not in newRobots:
                retiredRobots.append(robotName)

        self.activeRobotsLock.acquire()
        for robotName in retiredRobots:
            self.activeRobots[robotName]['subscriber'].unregister()
            self.activeRobots.pop(robotName, None)
        self.activeRobotsLock.release()
        rospy.loginfo('Active robots: {}'.format(self.activeRobots.keys()))

    def getHeatmap(self):
        occupancyMap = np.zeros(self.gridShape, dtype=bool)
        new_map = np.zeros(self.gridShape)
        activePositions = []
        self.activeRobotsLock.acquire()
        for robotName in self.activeRobots.keys():
            if self.activeRobots[robotName].get('pose') is not None:
                activePositions.append(self.activeRobots[robotName]['pose'])
        self.activeRobotsLock.release()

        for pose in activePositions:
            point = (pose.position.x, pose.position.y)
            cells = self.csm.convertPointToCells(point)
            firstOccupiedCell = self.csm.get_cell_containing_center(point)
            for (r, c) in cells:
                new_map[r][c] = 1
            occupancyMap[firstOccupiedCell] = True
        currentTime = datetime.datetime.now()
        timeDifference = currentTime - self.lastHeatmapCalaculatedTime
        self.lastHeatmapCalaculatedTime = currentTime
        deltaTime = timeDifference.total_seconds()  # time in seconds
        final_map = ETA * deltaTime * self.previousMap + new_map
        self.previousMap = final_map
        return occupancyMap, final_map

    def run(self):
        rate = rospy.Rate(HEATMAP_PUBLISH_RATE)
        iteration = 1
        while not rospy.is_shutdown():
            occupancyMap, heatmap = self.getHeatmap()
            self.heatmapPublisher.publish(heat_values=heatmap.flatten(), rows=self.gridShape[0], columns=self.gridShape[1])
            self.occupancyPublisher.publish(occupancy_values=occupancyMap.flatten(), rows=self.gridShape[0], columns=self.gridShape[1])
            rospy.loginfo('Iteration {}: heatmap published!'.format(iteration))
            iteration += 1
            rate.sleep()


def calculateHeatmap():
    mip = MapInformationProvider()
    heatmap = Heatmap(mip.numRowsInGrid, mip.numColumnsInGrid)
    rospy.loginfo('Heatmap node is running...')
    heatmap.run()


if __name__ == "__main__":
    calculateHeatmap()
