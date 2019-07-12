import rospy
import time
import numpy as np
from enum import Enum
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sorting_robot.msg import State, Pickup
from sorting_robot.srv import Path, PathInPickup, PathToBin, GetPickup, MakePickup
from sequencer import Sequencer
from ..utils import CoordinateSpaceManager

'''
The BFSM acts as the overall highest level of control of the program. It purely deals with the different
states the robot can exist in.
- "SELECT_PICKUP" - The init state and also the state where the robot has to select a pickup station 
                    and go receive the package.
- "GO_TO_PICKUP" - Receive the path to pickup station from the path planner and call the sequencer to move.
- "MAKING_PICKUP" - Reached the pickup station - Receive the package and the bin address.
- "GO_TO_BIN - Receive the path to bin from the path planner and go to the bin by calling the sequencer"
- "MAKE_THE_DROP" - Drop the package in the bin and change state to SELECT_PICKUP. 

The BFSM creates an instance of the sequencer and calls the follow_path method to move the robot to the goal
The BFSM communicates with the path planner and the pickup manager
/path - Get the path to the pickup point from the path planner.
/path_to_bin - Get the path to the bin from the path planner.
/pickup_location - Get the pickup point location from the pickup manager
/make_pickup - Make the pickup at the pickup point from the pickup manager

The charging parts can be added here eventually.
'''


class RobotState(Enum):
    GO_TO_PICKUP = 0
    SELECT_PICKUP = 1
    MAKE_THE_PICKUP = 2
    GO_TO_BIN = 3
    MAKE_THE_DROP = 4
    GO_TO_CHARGE = 5
    SELECT_CHARGE = 6
    CHARGING = 7


class BFSM:
    def __init__(self, robot_name):
        self.node_name = robot_name + '_bfsm'
        rospy.init_node(self.node_name, anonymous=False, log_level=rospy.INFO)
        self.state = RobotState.SELECT_PICKUP
        self.name = robot_name
        self.pose = State()
        self.pickup_location = State()
        self.bin_location = State()
        self.pickup_id = None
        self.ready = False
        self.csm = CoordinateSpaceManager()
        self.sequencer = Sequencer(robot_name)
        self.pose_subscriber = rospy.Subscriber('/' + robot_name + '/odom', Odometry, self.odom_callback)
        self.path_service = rospy.ServiceProxy('/path', Path)
        self.bin_service = rospy.ServiceProxy('/path_to_bin', PathToBin)
        self.pickup_service = rospy.ServiceProxy('/pickup_location', GetPickup)
        self.make_pickup_service = rospy.ServiceProxy('/make_pickup', MakePickup)

    def odom_callback(self, data):
        row, col, directionInDegrees = self.csm.convertPoseToState(data.pose.pose)
        self.pose.row = row
        self.pose.col = col
        self.pose.direction = directionInDegrees
        self.ready = True

    def run(self):
        while(self.ready is False):
            continue
        while not rospy.is_shutdown():
            if(self.state == RobotState.GO_TO_PICKUP):
                path = self.path_service(self.pose, self.pickup_location)
                rospy.loginfo("Received path from the planner to pickup")
                self.sequencer.follow_path(path.path)
                self.state = RobotState.MAKE_THE_PICKUP
            elif(self.state == RobotState.SELECT_PICKUP):
                pickup_message = self.pickup_service(String(self.name)).pickup
                rospy.loginfo("Received the address of the pickup")
                self.pickup_location = pickup_message.location
                self.pickup_id = pickup_message.pickup_id
                self.state = RobotState.GO_TO_PICKUP
            elif(self.state == RobotState.MAKE_THE_PICKUP):
                rospy.loginfo("Making the Pickup")
                self.bin_location = self.make_pickup_service(self.pickup_id, String(self.name)).location
                time.sleep(2)
                rospy.loginfo("Received the address of the bin")
                self.state = RobotState.GO_TO_BIN
            elif(self.state == RobotState.GO_TO_BIN):
                path = self.bin_service(self.pose, self.bin_location)
                rospy.loginfo("Received path to the bin")
                self.sequencer.follow_path(path.path)
                self.state = RobotState.MAKE_THE_DROP
            elif(self.state == RobotState.MAKE_THE_DROP):
                rospy.loginfo("Making the drop")
                time.sleep(1)
                self.state = RobotState.SELECT_PICKUP
