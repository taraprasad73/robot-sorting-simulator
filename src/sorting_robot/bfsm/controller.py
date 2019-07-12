import math
import threading
import numpy as np
from enum import Enum
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from sorting_robot.msg import OccupancyMap
from sorting_robot.srv import GoalService, ReachedService
from ..utils import CoordinateSpaceManager, MapInformationProvider
# TODO from rospy.numpy_msg import numpy_msg http://wiki.ros.org/rospy_tutorials/Tutorials/numpy


'''
The controller has three states
idle - doing nothing
moving - is moving towards the current subgoal
reached - has reached the subgoal
waiting - waiting for the path in front of it to clear

The contolller communicates with the sequencer using two services.
/subgoal service is called by the sequencer to provide the next subgoal to the controller
/reached_subgoal is called by the controller to give an acknowledgement to the sequencer

Some of the terms used are explained below:
 - LINEAR_VELOCITY_MULTIPLIER - Constant for linear velocity of the robot
 - ANGUALR_VELOCITY_MULTIPLIER - Constant for angular velocity of the robot
 - dg - Distance of the robot from the goal
 - dx - Distance of the robot from the goal along x-axis
 - dy - Distance of the robot from the goal along y-axis
 - angleToGoal - Angle to goal
 - angleToTurn - Difference between the robot orientation and the angleToGoal.
 - angleToRotate - Angle to rotate so as to aling the robot with the goal's orientation

WARNING: Using debug mode for log level will consume large amount of disk space.
'''

MAX_LINEAR_VELOCITY = 1.4  # in m/s
GOAL_REACHED_TOLERANCE = 0.01  # in m
ANGLE_REACHED_TOLERANCE = 0.01  # in radians
ANGLE_CORRECTION_TOLERANCE = 0.01  # in radians
LINEAR_VELOCITY_MULTIPLIER = 2
ANGULAR_VELOCITY_MULTIPLIER = 1
VELOCITY_PUBLISH_FREQUENCY = 50
NUMBER_OF_CELLS_TO_SCAN = 20


class RobotState(Enum):
    IDLE = 0
    MOVING_TO_MAIN_GOAL = 1
    REACHED_MAIN_GOAL = 2
    WAITING_FOR_CLEARANCE = 3
    MOVING_TO_TEMPORARY_GOAL = 4


class Controller:
    def __init__(self, robot_name):
        # create node
        self.node_name = robot_name + '_controller'
        rospy.init_node(self.node_name, anonymous=False, log_level=rospy.INFO)

        # robot's info
        self.currentPosition = Pose()
        self.currentGoal = None
        self.currentRow = None
        self.currentCol = None
        self.directionInDegrees = None
        self.deltaRow = None
        self.deltaCol = None
        self.mainGoal = None
        self.velocity = Twist()
        self.robotHeading = 0
        self.reached_count = 0
        self.robotName = robot_name
        self.robotState = RobotState.IDLE

        # utilities classes
        self.mip = MapInformationProvider()
        self.csm = CoordinateSpaceManager()

        # subscribe to the occupancy_map for collision prevention
        self.occupancy_subscriber = rospy.Subscriber('/occupancy_map', OccupancyMap, self.occupancy_callback)
        self.occupancyMap = None
        self.occupancyMapLock = threading.Lock()

        # subscribe to the odom to get the position of the robot
        # TODO add lock variables if needed
        self.pose_subscriber = rospy.Subscriber('/' + robot_name + '/odom', Odometry, self.odom_callback)

        # provide a service to receive goal from controller
        # TODO add lock variables if needed
        self.goal_service = rospy.Service('/' + robot_name + '/subgoal', GoalService, self.receive_goal)

        # consume reached_subgoal service to provide acknowledgement
        self.reached_service = rospy.ServiceProxy('/' + robot_name + '/reached_subgoal', ReachedService)

        # publish the velocity of the robot to cmd_vel topic
        self.velocityPublisher = rospy.Publisher('/' + robot_name + '/cmd_vel', Twist, queue_size=10)
        self.velocityPublishRate = rospy.Rate(VELOCITY_PUBLISH_FREQUENCY)

    def occupancy_callback(self, data):
        rows = data.rows
        cols = data.columns
        self.occupancyMapLock.acquire()
        # TODO use rospy.numpy_msg instead of converting list to a numpy array
        self.occupancyMap = np.array(data.occupancy_values, dtype=bool).reshape((rows, cols))
        self.occupancyMapLock.release()

    def odom_callback(self, data):
        self.currentPosition = data.pose.pose
        self.robotHeading = euler_from_quaternion([self.currentPosition.orientation.x, self.currentPosition.orientation.y,
                                                   self.currentPosition.orientation.z, self.currentPosition.orientation.w])[2]
        self.currentRow, self.currentCol, self.directionInDegrees = self.csm.convertPoseToState(self.currentPosition)

    def set_current_goal(self, row, col, goal):
        self.currentGoalRow = row
        self.currentGoalCol = col
        self.currentGoal = goal
        rospy.loginfo('Current goal changed to: {} {}'.format(row, col))

    # This function should be invoked by the sequencer only when the robot is in the idle state.
    def receive_goal(self, data):
        if self.robotState != RobotState.IDLE:
            rospy.logwarn('The robot is not in idle state. Rejecting the request to go to ({}, {}, {})'.format(
                data.goal.row, data.goal.col, data.goal.direction))
            return False
        self.mainGoalRow = data.goal.row
        self.mainGoalCol = data.goal.col
        self.mainGoal = self.csm.getPoseFromGridCoordinates(data.goal.row, data.goal.col, data.goal.direction)
        rospy.loginfo("Received goal from sequencer: ({:0.3f} {:0.3f} {})".format(self.mainGoal.position.x,
                                                                                  self.mainGoal.position.y,
                                                                                  self.mainGoal.orientation.z))
        self.set_current_goal(self.mainGoalRow, self.mainGoalCol, self.mainGoal)

        # wait for the odom_callback to set this variable
        while(self.directionInDegrees is None):
            continue

        # deltaRow, deltaCol when added to current position progresses forward in the direction
        # from current position to goal
        self.deltaRow, self.deltaCol = 0, 0
        if self.directionInDegrees == 0:
            self.deltaCol = 1
        elif self.directionInDegrees == 90:
            self.deltaRow = -1
        elif self.directionInDegrees == 180:
            self.deltaCol = -1
        elif self.directionInDegrees == 270:
            self.deltaRow = 1

        self.robotState = RobotState.MOVING_TO_MAIN_GOAL
        return True

    # scan for obstacles till the current goal or NUMBER_OF_CELLS_TO_SCAN whichever is minimum
    def get_first_occupied_cell(self):
        occupiedCell = None
        row = self.currentRow
        col = self.currentCol
        rospy.logdebug('Current Pos: {} {} '
                       'Goal Pos: {} {} '
                       'delRow:{} delCol:{}'.format(self.currentRow, self.currentCol,
                                                    self.currentGoalRow, self.currentGoalCol,
                                                    self.deltaRow, self.deltaCol))
        if not(row == self.currentGoalRow and col == self.currentGoalCol):
            self.occupancyMapLock.acquire()
            for cellsScanned in range(NUMBER_OF_CELLS_TO_SCAN):
                row += self.deltaRow
                col += self.deltaCol
                rospy.logdebug('Cell scanned ({},{}). Occupied: {}'.format(row, col, self.occupancyMap[row][col]))
                if self.occupancyMap[row][col]:
                    occupiedCell = (row, col)
                    break
                if row == self.currentGoalRow and col == self.currentGoalCol:
                    break
            self.occupancyMapLock.release()
        if occupiedCell is not None:
            rospy.logdebug('Occupied cell: ({},{})'.format(occupiedCell[0], occupiedCell[1]))
        return occupiedCell

    def get_nearest_non_intersection_cell(self, firstOccupiedCell):
        row, col = firstOccupiedCell
        row -= self.deltaRow
        col -= self.deltaCol
        while(self.mip.isIntersection(row, col)):
            row -= self.deltaRow
            col -= self.deltaCol
            # if after going back, the cell is the one with the robot in it, and is still an intersection
            if row == self.currentRow and col == self.currentCol:
                return None
        return (row, col)

    def stop_the_robot(self):
        self.velocity.angular.z = 0
        self.velocity.linear.x = 0
        self.velocityPublisher.publish(self.velocity)

    def run(self):
        rospy.loginfo('{} is ready'.format(self.node_name))
        while not rospy.is_shutdown():
            if self.robotState == RobotState.IDLE:
                continue
            elif self.robotState == RobotState.WAITING_FOR_CLEARANCE:
                if self.get_first_occupied_cell() is None:
                    self.set_current_goal(self.mainGoalRow, self.mainGoalCol, self.mainGoal)
                    self.robotState = RobotState.MOVING_TO_MAIN_GOAL
                    rospy.loginfo('Clearance received! Going to main goal...')
            elif self.robotState == RobotState.MOVING_TO_MAIN_GOAL:
                firstOccupiedCell = self.get_first_occupied_cell()
                if firstOccupiedCell is not None:
                    self.avoid_obstacle(firstOccupiedCell)
                else:
                    dx, dy, distanceToGoal = self.get_distances()
                    if(distanceToGoal < GOAL_REACHED_TOLERANCE):
                        self.robotState = RobotState.REACHED_MAIN_GOAL
                    else:
                        self.adjust_velocity(dx, dy)
            elif self.robotState == RobotState.MOVING_TO_TEMPORARY_GOAL:
                firstOccupiedCell = self.get_first_occupied_cell()
                if firstOccupiedCell is not None:
                    self.avoid_obstacle(firstOccupiedCell)
                else:
                    dx, dy, distanceToGoal = self.get_distances()
                    if(distanceToGoal < GOAL_REACHED_TOLERANCE):
                        self.stop_the_robot()
                        self.robotState = RobotState.WAITING_FOR_CLEARANCE
                        rospy.loginfo('Reached temporary goal. Waiting for clearance...')
                    else:
                        self.adjust_velocity(dx, dy)
            elif self.robotState == RobotState.REACHED_MAIN_GOAL:
                self.alignWithGoalOrientation()
                self.stop_the_robot()
                rospy.loginfo('Goal orientation reached. Job finished! Going to idle state.')
                while(True):
                    status = self.reached_service(True)
                    if status is False:
                        rospy.logwarn('Sub goal reached acknowledgement not received by sequencer. Retrying...')
                    else:
                        break
                self.robotState = RobotState.IDLE

    def avoid_obstacle(self, firstOccupiedCell):
        rospy.loginfo('Obstacle encountered at {} while {}'.format(firstOccupiedCell, self.robotState.name))
        nearestNonIntersectionCell = self.get_nearest_non_intersection_cell(firstOccupiedCell)
        if nearestNonIntersectionCell is not None:
            row, col = nearestNonIntersectionCell
            # use the current direction of the robot for the temporary goal
            goal = self.csm.getPoseFromGridCoordinates(row, col, self.directionInDegrees)
            self.set_current_goal(row, col, goal)
            self.robotState = RobotState.MOVING_TO_TEMPORARY_GOAL
            rospy.loginfo('Going to nearest non-intersection cell: {}.'.format(nearestNonIntersectionCell))
        else:
            self.stop_the_robot()
            self.robotState = RobotState.WAITING_FOR_CLEARANCE
            rospy.logwarn("Robot had to stop at an intersection cell."
                          "This case should be prevented from happening.")
            # TODO move to a non-intersection cell as soon as possible

    def get_distances(self):
        dx = self.currentGoal.position.x - self.currentPosition.position.x
        dy = self.currentGoal.position.y - self.currentPosition.position.y
        distanceToGoal = math.sqrt(dx * dx + dy * dy)
        return dx, dy, distanceToGoal

    def adjust_velocity(self, dx, dy):
        angleToGoal = np.arctan2(dy, dx)
        angleToTurn = np.arctan2(math.sin(angleToGoal - self.robotHeading),
                                 math.cos(angleToGoal - self.robotHeading))
        rospy.logdebug('pose: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}]  goal: [x:{:0.3f}, y:{:0.3f},th:{:0.3f}]'
                       ' a2g: {:0.5f}'.format(
                           self.currentPosition.position.x, self.currentPosition.position.y,
                           self.robotHeading, self.currentGoal.position.x, self.currentGoal.position.y,
                           self.currentGoal.orientation.z, angleToTurn))
        if(abs(angleToTurn) >= ANGLE_CORRECTION_TOLERANCE):
            self.velocity.angular.z = ANGULAR_VELOCITY_MULTIPLIER * angleToTurn
            self.velocity.linear.x = 0
        else:
            self.velocity.linear.x = min(LINEAR_VELOCITY_MULTIPLIER * math.sqrt(dx * dx + dy * dy),
                                         MAX_LINEAR_VELOCITY)
            self.velocity.angular.z = 0
        self.velocityPublisher.publish(self.velocity)
        self.velocityPublishRate.sleep()

    def alignWithGoalOrientation(self):
        # align the robot w.r.t. the orientation of the current goal
        while(not rospy.is_shutdown()):
            angleToRotate = angle_difference(self.currentGoal.orientation.z, self.robotHeading)
            rospy.logdebug('pose: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}]  goal: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}] a2r: {:0.5f}'.format(
                self.currentPosition.position.x, self.currentPosition.position.y, self.robotHeading, self.currentGoal.position.x,
                self.currentGoal.position.y, self.currentGoal.orientation.z, angleToRotate))
            if(abs(angleToRotate) < ANGLE_REACHED_TOLERANCE):
                break
            self.velocity.angular.z = ANGULAR_VELOCITY_MULTIPLIER * angleToRotate
            self.velocity.linear.x = 0.0
            self.velocityPublisher.publish(self.velocity)
            self.velocityPublishRate.sleep()


def angle_difference(angle1, angle2):
    if(abs(angle1 - angle2) <= math.pi):
        return angle1 - angle2
    elif((angle1 - angle2) > math.pi):
        return -(2 * math.pi - (angle1 - angle2))
    else:
        return 2 * math.pi + (angle1 - angle2)
