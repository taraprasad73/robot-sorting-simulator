import os
import yaml

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
ROBOT_CONFIGURATION_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/stdr_data/robots/pandora_robot.yaml'


class RobotInfo:
    @staticmethod
    def getRobotRadiusInMeters():
        try:
            with open(ROBOT_CONFIGURATION_FILE_LOCATION, 'r') as stream:
                robotInfo = yaml.safe_load(stream)
        except IOError:
            print(ROBOT_CONFIGURATION_FILE_LOCATION + " doesn't exist.");
            raise IOError("robot config file doesn't exist.")
        else:
            return 1000 * robotInfo['robot']['robot_specifications'][0]['footprint']['footprint_specifications']['radius']

    @staticmethod
    def getAverageLinearSpeed():
        return 1

    @staticmethod
    def getAverageAngularSpeed():
        return 0.5
