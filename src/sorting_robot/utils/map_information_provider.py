import os
import rospy
import numpy as np
from ..map_generation.generate_map_config import Cell, Direction, CellType

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}_configuration.npy'


class MapInformationProvider:
    def __init__(self, mapName='map'):
        global CONFIG_FILE_LOCATION
        CONFIG_FILE_LOCATION = CONFIG_FILE_LOCATION.format(mapName)
        try:
            mapConfiguration = np.load(CONFIG_FILE_LOCATION).item()
        except IOError:
            rospy.logerror("{} doesn't exist. Run the following command to create it:\nrosrun sorting_robot"
                           "generate_map_config".format(CONFIG_FILE_LOCATION))
            raise IOError("Config file doesn't exist.")
        else:
            self.numRowsInGrid = mapConfiguration['num_rows']
            self.numColumnsInGrid = mapConfiguration['num_columns']
            self.cellLength = mapConfiguration['cell_length_in_meters']
            self.grid = mapConfiguration['grid']
            self.pickups = mapConfiguration['pickups']
            self.pickupQueueSize = mapConfiguration['pickup_queue_size']

    def isIntersection(self, row, col):
        cellType = self.grid[row][col].cellType
        if(cellType == CellType.STREET_STREET_INTERSECTION or cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION or
           cellType == CellType.HIGHWAY_STREET_INTERSECTION):
            return True
        return False
