import os
import rospy
import numpy as np
from ..map_generation.generate_map_config import Cell, Direction, CellType

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
if not os.environ.get('SORTING_ROBOT_MAP'):
    logging.error('SORTING_ROBOT_MAP environment variable is not set.')
    exit(1)
MAP_NAME = os.environ['SORTING_ROBOT_MAP']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}_configuration.npy'.format(MAP_NAME, MAP_NAME)
BINARY_GRID_IMAGE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}_binary_grid.png'.format(MAP_NAME, MAP_NAME)
BINARY_GRID_IMAGE_LOCATION_STDR = CATKIN_WORKSPACE + '/src/sorting_robot/data/stdr_data/maps/{}_binary_grid.png'.format(MAP_NAME, MAP_NAME)
MAP_IMAGE_FILE_SVG_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}.svg'.format(MAP_NAME, MAP_NAME)
MAP_IMAGE_FILE_PNG_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}.png'.format(MAP_NAME, MAP_NAME)
MAP_LEGEND_FILE_SAVE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}_legend.png'.format(MAP_NAME, MAP_NAME)
GRAPH_PICKLED_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}_graph.gpickle'.format(MAP_NAME, MAP_NAME)
GRAPH_IMAGE_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}/{}_graph.svg'.format(MAP_NAME, MAP_NAME)

if not os.path.exists(CATKIN_WORKSPACE + '/src/sorting_robot/data/spawn_scripts/'):
    os.makedirs(CATKIN_WORKSPACE + '/src/sorting_robot/data/spawn_scripts/')
SPAWN_LOCATIONS_SCRIPT_FILE = CATKIN_WORKSPACE + '/src/sorting_robot/data/spawn_scripts/spawn_robots_on_{}.sh'.format(MAP_NAME, MAP_NAME)


class MapInformationProvider:
    def __init__(self):
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
