import os
import rospy
import numpy as np
from enum import Enum
from sorting_robot.srv import GoalService, ReachedService

'''
The sequencer depends upon these three states of the robot
    INIT - starting state
    MOVING - moving towards the goal
    REACHED - has reached the goal

The sequencer communicates with the traffic manager and the controller as follows - 
    /subgoal service is called by the sequencer to provide the next subgoal to the controller
    /reached_subgoal is called by the controller to give an acknowledgement to the sequencer

The process_path() function plays an important role in breaking down the sequence of steps provided
by the path_planner into a set of subgoals.
 - Every turn/intersection becomes a subgoal

The follow_path() function actually communicates with the controller by providing subgoals and receiving
acknoledgements for the same. This function is called by the BFSM.
'''


class SequencerState(Enum):
    WAITING_FOR_GOAL_REACHED_ACK = 1
    GOAL_REACHED_ACK_RECEIVED = 2
    GOAL_NOT_REACHED_ACK_RECEIVED = 3


class Sequencer:
    def __init__(self, robotName):
        self.sequencerState = None
        self.robotName = robotName
        self.reached_service = rospy.Service('/' + robotName + '/reached_subgoal', ReachedService, self.received_ack)
        self.goal_service = rospy.ServiceProxy('/' + robotName + '/subgoal', GoalService)

    def received_ack(self, data):
        if data.reached is True:
            self.sequencerState = SequencerState.GOAL_REACHED_ACK_RECEIVED
            rospy.loginfo("Subgoal reached acknowledgement received by sequencer.")
        else:
            self.sequencerState = SequencerState.GOAL_NOT_REACHED_ACK_RECEIVED
            rospy.loginfo("Subgoal not reached acknowledgement received by sequencer.")
        return True

    def is_same_cell(self, prev, curr):
        if(prev.row == curr.row and prev.col == curr.col):
            return True
        return False

    # breaks down the path into straight movements and turns
    def process_path(self, path):
        if(len(path) <= 1):
            return path
        subgoals = []
        previousCellOnPath = path[0]
        for i in range(1, len(path)):
            if(self.is_same_cell(previousCellOnPath, path[i])):
                subgoals.append(path[i])
            previousCellOnPath = path[i]
        return subgoals

    # move the robot according to the path given, by interacting with the controller
    def follow_path(self, path):
        subgoals = self.process_path(path)
        for subgoal in subgoals:
            if rospy.is_shutdown():
                break
            rospy.loginfo('current subgoal: {} {} {}'.format(subgoal.row, subgoal.col, subgoal.direction))
            ack = self.goal_service(subgoal)
            if ack is False:
                rospy.logwarn('sub goal: {} {} {} not received by controller.'.format(subgoal.row,
                                                                                      subgoal.col,
                                                                                      subgoal.direction))
            self.sequencerState = SequencerState.WAITING_FOR_GOAL_REACHED_ACK
            while(not rospy.is_shutdown() and self.sequencerState == SequencerState.WAITING_FOR_GOAL_REACHED_ACK):
                continue

            if self.sequencerState == SequencerState.GOAL_NOT_REACHED_ACK_RECEIVED:
                # TODO Examine this case. Will it ever happen? How to handle it?
                pass
