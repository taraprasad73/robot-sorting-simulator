import os;
import time;
import matplotlib.pyplot as plt
import numpy as np;
import threading;
import rospy;
from sorting_robot.msg import OccupancyMap, TrafficLight;
from sorting_robot.srv import TrafficService;
from ..map_generation.generate_map_config import Cell, Direction, Turn, CellType;

'''
The convention for naming the states are as follows - The first letter signifies the traffic light in the horizontal lane
and the second light signified the traffic light in the vertical direction.
GR - Horizontal Green and Vertical Red
YR - Horizontal Yellow and Vertical Red
RG - Horizontal Red and Vertical Green
RY - Horizontal Red and Vertical Yellow
'''

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ['CATKIN_WORKSPACE']:
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'
occupancy_map = [];


class StreetSignal:
    def __init__(self, row, col, rows, cols, directions, k, wait_time):
        self.row = row;
        self.col = col;
        self.rows = rows;
        self.cols = cols;
        self.k = k;
        self.directions = directions;
        self.horizontal = Direction.LEFT if(Direction.LEFT in self.directions) else Direction.RIGHT;
        self.vertical = Direction.UP if(Direction.UP in self.directions) else Direction.DOWN;
        self.states = ['GR', 'YR', 'RG', 'RY'];
        self.curr_state = self.states[0];
        self.wait_time = wait_time;

    def get_signal(self):
        signal = TrafficLight();
        signal.left = False;
        signal.right = False;
        signal.up = False;
        signal.down = False;
        if(self.curr_state == 'GR'):
            if(self.horizontal == Direction.LEFT):
                signal.left = True;
            else:
                signal.right = True;
        elif(self.curr_state == 'RG'):
            if(self.vertical == Direction.UP):
                signal.up = True;
            else:
                signal.down = True;
        return signal;

    def is_empty(self):
        return occupancy_map[self.row][self.col];

    def vertical_waiting(self):
        if(self.vertical == Direction.UP):
            count = 0;
            for i in range(self.row + 1, min(self.row + self.k + 1, self.rows)):
                if(occupancy_map[i][self.col] is True):
                    count += 1;
            return count;
        else:
            count = 0;
            for i in range(self.row - 1, max(self.row - self.k - 1, -1), -1):
                if(occupancy_map[i][self.col] is True):
                    count += 1;
            return count;

    def horizontal_waiting(self):
        if(self.horizontal == Direction.LEFT):
            count = 0;
            for i in range(self.col + 1, min(self.col + self.k + 1, self.cols)):
                if(occupancy_map[self.row][i] is True):
                    count += 1;
            return count
        else:
            count = 0;
            for i in range(self.col - 1, min(self.col - self.k - 1, -1), -1):
                if(occupancy_map[self.row][i] is True):
                    count += 1;
            return count;

    def run(self):
        while not rospy.is_shutdown():
            if(self.curr_state == 'GR'):
                if(self.horizontal_waiting() == 0 and self.vertical_waiting() == self.k):
                    self.curr_state = 'YR';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'YR';
            elif(self.curr_state == 'YR'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'RG';
            elif(self.curr_state == 'RG'):
                if(self.vertical_waiting() == 0 and self.horizontal_waiting() == self.k):
                    self.curr_state = 'RY';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'RY';
            elif(self.curr_state == 'RY'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'GR';


class HighwaySignal:
    def __init__(self, row, col, rows, cols, k, wait_time):
        self.row = row;
        self.col = col;
        self.rows = rows;
        self.cols = cols;
        self.k = k;
        self.states = ['GR', 'YR', 'RG', 'RY'];
        self.curr_state = self.states[0];
        self.wait_time = wait_time;

    def get_signal(self):
        signal = TrafficLight();
        signal.left = False;
        signal.right = False;
        signal.up = False;
        signal.down = False;
        if(self.curr_state == 'GR'):
            signal.left = True;
            signal.right = True;
        elif(self.curr_state == 'RG'):
            signal.up = True;
            signal.down = True;
        return signal;

    def is_empty(self):
        if(occupancy_map[self.row][self.col] is True or occupancy_map[self.row + 1][self.col] is True or
                occupancy_map[self.row + 1][self.col + 1] is True or occupancy_map[self.row][self.col + 1] is True):
            return False;
        return True;

    def vertical_waiting(self):
        count = 0;
        for i in range(self.row + 1, min(self.row + self.k + 1, self.rows)):
            if(occupancy_map[i][self.col] is True):
                count += 1;
        for i in range(self.row - 1, max(self.row - self.k - 1, -1), -1):
            if(occupancy_map[i][self.col + 1] is True):
                count += 1;
        return count;

    def horizontal_waiting(self):
        count = 0;
        for i in range(self.col + 1, min(self.col + self.k + 1, self.cols)):
            if(occupancy_map[self.row + 1][i] is True):
                count += 1;
        for i in range(self.col - 1, min(self.col - self.k - 1, -1), -1):
            if(occupancy_map[self.row][i] is True):
                count += 1;
        return count;

    def run(self):
        while not rospy.is_shutdown():
            if(self.curr_state == 'GR'):
                if(self.horizontal_waiting() == 0 and self.vertical_waiting() == self.k * 2):
                    self.curr_state = 'YR';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'YR';
            elif(self.curr_state == 'YR'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'RG';
            elif(self.curr_state == 'RG'):
                if(self.vertical_waiting() == 0 and self.horizontal_waiting() == self.k * 2):
                    self.curr_state = 'RY';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'RY';
            elif(self.curr_state == 'RY'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'GR';


class HybridSignal:
    def __init__(self, row, col, rows, cols, directions, k, wait_time):
        self.row = row;
        self.col = col;
        self.rows = rows;
        self.cols = cols;
        self.k = k;
        self.directions = directions;
        if(Direction.LEFT in self.directions):
            self.horizontal = Direction.LEFT;
            self.vertical = Direction.UP;
        elif(Direction.RIGHT in self.directions):
            self.horizontal = Direction.RIGHT;
            self.vertical = Direction.DOWN;
        elif(Direction.UP in self.directions):
            self.horizontal = Direction.RIGHT;
            self.vertical = Direction.UP;
        elif(Direction.DOWN in self.directions):
            self.horizontal = Direction.LEFT;
            self.vertical = Direction.DOWN;
        self.states = ['GR', 'YR', 'RG', 'RY'];
        self.curr_state = self.states[0];
        self.wait_time = wait_time;

    def get_signal(self):
        signal = TrafficLight();
        signal.left = False;
        signal.right = False;
        signal.up = False;
        signal.down = False;
        if(self.curr_state == 'GR'):
            if(self.horizontal == Direction.LEFT):
                signal.left = True;
            else:
                signal.right = True;
        elif(self.curr_state == 'RG'):
            if(self.vertical == Direction.UP):
                signal.up = True;
            else:
                signal.down = True;
        return signal;

    def is_empty(self):
        return occupancy_map[self.row][self.col];

    def vertical_waiting(self):
        if(self.vertical == Direction.UP):
            count = 0;
            for i in range(self.row + 1, min(self.row + self.k + 1, self.rows)):
                if(occupancy_map[i][self.col] is True):
                    count += 1;
            return count;
        else:
            count = 0;
            for i in range(self.row - 1, max(self.row - self.k - 1, -1), -1):
                if(occupancy_map[i][self.col] is True):
                    count += 1;
            return count;

    def horizontal_waiting(self):
        if(self.horizontal == Direction.LEFT):
            count = 0;
            for i in range(self.col + 1, min(self.col + self.k + 1, self.cols)):
                if(occupancy_map[self.row][i] is True):
                    count += 1;
            return count
        else:
            count = 0;
            for i in range(self.col - 1, min(self.col - self.k - 1, -1), -1):
                if(occupancy_map[self.row][i] is True):
                    count += 1;
            return count;

    def run(self):
        while not rospy.is_shutdown():
            if(self.curr_state == 'GR'):
                if(self.horizontal_waiting() == 0 and self.vertical_waiting() == self.k):
                    self.curr_state = 'YR';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'YR';
            elif(self.curr_state == 'YR'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'RG';
            elif(self.curr_state == 'RG'):
                if(self.vertical_waiting() == 0 and self.horizontal_waiting() == self.k):
                    self.curr_state = 'RY';
                else:
                    time.sleep(self.wait_time);
                    self.curr_state = 'RY';
            elif(self.curr_state == 'RY'):
                while(self.is_empty() is False):
                    continue;
                self.curr_state = 'GR';

class TrafficManager:
    def __init__(self, k, data):
        rospy.init_node('traffic_manager', anonymous=False);
        self.data = data;
        rows, cols = data.shape;
        global occupancy_map;
        for i in range(0, rows):
            occupancy_map.append([False] * cols);
        self.k = k;
        self.wait_time = 3;
        self.traffic_signals = dict();
        self.threads = [];
        for i in range(rows):
            for j in range(cols):
                if(data[i][j].cellType == CellType.STREET_STREET_INTERSECTION):
                    traffic_signal = StreetSignal(i, j, rows, cols, data[i][j].directions, self.k, self.wait_time);
                    self.traffic_signals[(i, j)] = traffic_signal;
                elif(data[i][j].cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION):
                    if(self.topleft(i, j, rows, cols) is False):
                        continue;
                    traffic_signal = HighwaySignal(i, j, rows, cols, self.k, self.wait_time);
                    self.traffic_signals[(i, j)] = traffic_signal;
                    self.traffic_signals[(i, j + 1)] = traffic_signal;
                    self.traffic_signals[(i + 1, j)] = traffic_signal;
                    self.traffic_signals[(i + 1, j + 1)] = traffic_signal;
                elif(data[i][j].cellType == CellType.HIGHWAY_STREET_INTERSECTION):
                    if(self.traffic_clash(i,j,rows,cols)==True):
                        traffic_signal = HybridSignal(i, j, rows, cols, data[i][j].directions, self.k, self.wait_time);
                        self.traffic_signals[(i, j)] = traffic_signal;
        for key in self.traffic_signals.keys():
            thread = threading.Thread(target=self.traffic_signals[key].run);
            thread.start();
            self.threads.append(thread);
        self.map_subscriber = rospy.Subscriber('/occupancy_map', OccupancyMap, self.map_callback);
        self.service = rospy.Service('/traffic', TrafficService, self.get_traffic_signal);

    def get_traffic_signal(self, req):
        x, y = req.location.row, req.location.col;
        return self.traffic_signals[(x, y)].get_signal();

    def map_callback(self):
        global occupancy_map;
        rows = self.data.rows;
        cols = self.data.cols;
        new_map = [];
        k = 0;
        for i in range(0, rows):
            temp = [];
            for j in range(0, cols):
                temp.append(self.data.occupancy_values[k]);
                k += 1;
            new_map.append(temp);
        occupancy_map = new_map;

    def topleft(self, i, j, rows, cols):
        def bounds(i, j, rows, cols):
            return i >= 0 and j >= 0 and i < rows and j < cols;

        if(bounds(i - 1, j, rows, cols) is True and self.data[i - 1][j].cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION):
            return False;
        if(bounds(i, j - 1, rows, cols) is True and self.data[i][j - 1].cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION):
            return False;
        return True;

    def traffic_clash(self, i, j, rows, cols):
        def bounds(i, j, rows, cols):
            return i >= 0 and j >= 0 and i < rows and j < cols;

        if(Direction.LEFT in self.data[i][j].directions):
            if(bounds(i+1,j,rows,cols) and Direction.UP in self.data[i+1][j].directions);
                return True;
            return False;
        elif(Direction.RIGHT in self.data[i][j].directions):
            if(bounds(i-1,j,rows,cols) and Direction.DOWN in self.data[i-1][j].directions);
                return True;
            return False;
        elif(Direction.UP in self.data[i][j].directions):
            if(bounds(i,j-1,rows,cols) and Direction.RIGHT in self.data[i][j-1].directions);
                return True;
            return False;
        elif(Direction.DOWN in self.data[i][j].directions):
            if(bounds(i,j+1,rows,cols) and Direction.LEFT in self.data[i][j+1].directions);
                return True;
            return False;

    def visualize(self):
        rows, cols = self.data.shape[0], self.data.shape[1];
        enlarge_ratio = 1;
        image = np.zeros((rows * enlarge_ratio, cols * enlarge_ratio, 3), dtype='uint8');
        # Traffic: Horizontal is green and Traffic Vertical is red
        plt.show();
        while not rospy.is_shutdown():
            for i in range(0, rows):
                for j in range(0, cols):
                    if(self.data[i][j].cellType == CellType.STREET_STREET_INTERSECTION):
                        signal = self.traffic_signals[(i, j)].get_signal();
                        if(signal.left is True or signal.right is True):
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([0, 255, 0]);
                        else:
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([255, 0, 0]);
                    elif(self.data[i][j].cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION):
                        signal = self.traffic_signals[(i, j)].get_signal();
                        if(signal.left is True):
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([0, 255, 0]);
                        else:
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([255, 0, 0]);
                    elif(self.data[i][j].cellType == CellType.HIGHWAY_STREET_INTERSECTION):
                        signal = self.traffic_signals[(i, j)].get_signal();
                        if(signal.left is True or signal.right is True):
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([0, 255, 0]);
                        else:
                            image[i * enlarge_ratio:(i + 1) * enlarge_ratio][j * enlarge_ratio:(j + 1) * enlarge_ratio] = np.array([255, 0, 0]);
            plt.imshow(image);
            plt.draw();
            plt.pause(1e-17);

    def close(self):
        for i in range(0, len(self.threads)):
            self.threads[i].join();

def traffic_manager(k):
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
        grid = mapConfiguration['grid'];
        manager = TrafficManager(k, grid);
        print("Traffic signals are running");
        #manager.visualize();
        #manager.close();
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config")


if __name__ == "__main__":
    traffic_manager(1);
