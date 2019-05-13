import argparse;
import rospy;
import re;
import numpy as np;
import os;
from geometry_msgs.msg import Pose;
from nav_msgs.msg import Odometry;
from sorting_robot.msg import HeatMap, OccupancyMap;

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'

HEATMAP_PUBLISH_RATE = 1;


def convertCoordinatesToCells(point, gridShape, cellLength):
    col = int(point.x // cellLength);
    row = gridShape[0] - int(point.y // cellLength) - 1;
    return row, col;


class Heatmap:
    def __init__(self, numRows, numColumns, cellLength):
        rospy.init_node('heatmap', anonymous=False);
        self.gridShape = (numRows, numColumns);
        self.cellLength = cellLength;
        self.heatmapPublisher = rospy.Publisher('/heat_map', HeatMap, queue_size=10);
        self.occupancyPublisher = rospy.Publisher('/occupancy_map', OccupancyMap, queue_size=10);
        self.subscribers = [];
        self.positions = {};
        self.findTopics();
        self.eta = 0.1;
        self.previousMap = np.zeros((numRows, numColumns));

    def callback(self, data, name):
        self.positions[name] = data.pose.pose;

    def findTopics(self):
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
            x, y = convertCoordinatesToCells(self.positions[key].position, self.gridShape, self.cellLength);
            new_map[x][y] = 1;
            occupancyMap[x][y] = True;
        final_map = self.eta * self.previousMap + (1 - self.eta) * new_map;
        self.previousMap = final_map;
        return occupancyMap, final_map;

    def run(self):
        rate = rospy.Rate(HEATMAP_PUBLISH_RATE);
        while not rospy.is_shutdown():
            occupancyMap, heatmap = self.getHeatmap();
            self.heatmapPublisher.publish(heat_values=heatmap.flatten(), rows=self.gridShape[0], columns=self.gridShape[1]);
            self.occupancyPublisher.publish(occupancy_values=occupancyMap.flatten(), rows=self.gridShape[0], cols=self.gridShape[1]);
            rate.sleep();


def calculateHeatmap():
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
        heatmap = Heatmap(mapConfiguration['num_rows'], mapConfiguration['num_columns'], mapConfiguration['cell_length_in_meters']);
        heatmap.run();
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config");


if __name__ == "__main__":
    calculateHeatmap();
