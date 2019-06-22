import argparse;
import rospy;
import re;
import os;
import threading
import numpy as np;
from geometry_msgs.msg import Pose;
from nav_msgs.msg import Odometry;
from sorting_robot.msg import HeatMap, OccupancyMap;
from ..utils import CoordinateSpaceManager, RobotInfo


HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}_configuration.npy'

HEATMAP_PUBLISH_RATE = 2;
TOPIC_SEARCH_INTERVAL = 0.5;


class Heatmap:
    def __init__(self, mapName, numRows, numColumns):
        rospy.init_node('heatmap', anonymous=False);
        self.csm = CoordinateSpaceManager(mapName)
        self.gridShape = (numRows, numColumns);
        self.heatmapPublisher = rospy.Publisher('/heat_map', HeatMap, queue_size=10);
        self.occupancyPublisher = rospy.Publisher('/occupancy_map', OccupancyMap, queue_size=10);
        self.subscribers = [];
        self.positions = {};
        self.eta = 0.3;
        self.previousMap = np.zeros((numRows, numColumns));
        rospy.Timer(rospy.Duration(TOPIC_SEARCH_INTERVAL), self.findTopics)

    def callback(self, data, name):
        self.positions[name] = data.pose.pose;

    def findTopics(self, data):
        subscribers = [];
        topics = rospy.get_published_topics();
        for topic in topics:
            topic_name, topic_type = topic;
            match = re.search("/.*/odom", topic_name);
            if(match):
                check = re.search("/.*/", topic_name);
                name = check.group()[1:len(check.group()) - 1];
                subscribers.append(rospy.Subscriber(topic_name, Odometry, self.callback, name));
        self.subscribers = subscribers;

    def getHeatmap(self):
        occupancyMap = np.zeros(self.gridShape, dtype=bool);
        new_map = np.zeros(self.gridShape);
        for key in self.positions.keys():
            point = (self.positions[key].position.x, self.positions[key].position.y)
            cells = self.csm.convertPointToCells(point);
            for (r, c) in cells:
                new_map[r][c] = 1;
                occupancyMap[r][c] = True;
        final_map = self.eta * self.previousMap + new_map;
        self.previousMap = final_map;
        return occupancyMap, final_map;

    def run(self):
        rate = rospy.Rate(HEATMAP_PUBLISH_RATE);
        while not rospy.is_shutdown():
            occupancyMap, heatmap = self.getHeatmap();
            self.heatmapPublisher.publish(heat_values=heatmap.flatten(), rows=self.gridShape[0], columns=self.gridShape[1]);
            self.occupancyPublisher.publish(occupancy_values=occupancyMap.flatten(), rows=self.gridShape[0], columns=self.gridShape[1]);
            rate.sleep();


def calculateHeatmap(mapName):
    global CONFIG_FILE_LOCATION
    CONFIG_FILE_LOCATION = CONFIG_FILE_LOCATION.format(mapName)
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config");
    else:
        heatmap = Heatmap(mapName, mapConfiguration['num_rows'], mapConfiguration['num_columns']);
        print('Heatmap node is running...')
        heatmap.run();


if __name__ == "__main__":
    calculateHeatmap();
