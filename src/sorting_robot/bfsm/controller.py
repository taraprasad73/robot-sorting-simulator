import sys;
import math;
import rospy;
import numpy as np;
from std_msgs.msg import Int32;
from nav_msgs.msg import Odometry;
from geometry_msgs.msg import Pose, Twist;
from sensor_msgs.msg import LaserScan;
from tf.transformations import euler_from_quaternion;
from sorting_robot.msg import *;
from sorting_robot.srv import GoalService, ReachedService;

'''
The controller has three states
idle - doing nothing
moving - is moving towards the current subgoal
reached - has reached the subgoal

The contolller communicates with the sequencer using two services.
/subgoal service is called by the sequencer to provide the next subgoal to the controller
/reached_subgoal is called by the controller to give an acknowledgement to the sequencer

The moveToGoal() function which actually moves the robot to its goal is described below
LINEAR_VELOCITY_MULTIPLIER - Constant for linear velocity of the robot
ANGUALR_VELOCITY_MULTIPLIER - Constant for angular velocity of the robot
dg - Distance of the robot from the goal
dx - Distance of the robot from the goal along x-axis
dy - Distance of the robot from the goal along y-axis
angleToGoal - Angle to goal
angleToTurn - Difference between the robot orientation and the angleToGoal.
angleToRotate - Angle to rotate so as to aling the robot with the goal's orientation
The first while loop is for linear movement while the next is for pure angular motion.
'''

MAX_LINEAR_VELOCITY = 1  # in m/s
GOAL_REACHED_TOLERANCE = 0.001  # in m
ANGLE_REACHED_TOLERANCE = 0.001  # in radians
LINEAR_VELOCITY_MULTIPLIER = 0.5
ANGUALR_VELOCITY_MULTIPLIER = 0.5
VELOCITY_PUBLISH_FREQUENCY = 1000


class Controller:
    def __init__(self, name):
        self.node_name = name + '_controller'
        rospy.init_node(self.node_name, anonymous=False);
        self.pose = Pose();
        self.goal = Pose();
        self.velocity = Twist();
        self.robotHeading = 0;
        self.reached_count = 0;
        self.name = name;
        self.state = "idle";
        self.possible_states = ["idle", "moving", "reached"];
        self.pose_subscriber = rospy.Subscriber('/' + name + '/odom', Odometry, self.odom_callback);
        self.goal_service = rospy.Service('/' + name + '/subgoal', GoalService, self.receive_goal);
        self.reached_service = rospy.ServiceProxy('/' + name + '/reached_subgoal', ReachedService);
        self.velocityPublisher = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=10);
        self.velocityPublishRate = rospy.Rate(VELOCITY_PUBLISH_FREQUENCY)

    def odom_callback(self, data):
        self.pose = data.pose.pose;
        self.robotHeading = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,
                                                   self.pose.orientation.z, self.pose.orientation.w])[2];
        # print('odom callback: pose={} theta={}'.format(self.pose.position, self.theta))

    def laser_callback(self, data):
        self.laser_scan = list(data.ranges);

    def receive_goal(self, data):
        self.goal = data.goal
        print("Received goal from sequencer: ({:0.3f} {:0.3f} {})".format(self.goal.position.x, self.goal.position.y, self.goal.orientation.z));
        self.state = "moving";
        return 1;

    def angle_difference(self, angle1, angle2):
        if(abs(angle1 - angle2) <= math.pi):
            return angle1 - angle2;
        elif((angle1 - angle2) > math.pi):
            return -(2 * math.pi - (angle1 - angle2));
        else:
            return 2 * math.pi + (angle1 - angle2);

    def moveToGoal(self):
        # make the robot reach the position of the current goal
        while(not rospy.is_shutdown()):
            dx = self.goal.position.x - self.pose.position.x;
            dy = self.goal.position.y - self.pose.position.y;
            angleToGoal = np.arctan2(dy, dx);
            angleToTurn = np.arctan2(math.sin(angleToGoal - self.robotHeading), math.cos(angleToGoal - self.robotHeading));
            distanceToGoal = math.sqrt(dx * dx + dy * dy);
            # use logdebug here, otherwise disk space will run out
            print('pose: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}]  goal: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}] a2g: {:0.5f}'.format(
                self.pose.position.x, self.pose.position.y, self.robotHeading, self.goal.position.x,
                self.goal.position.y, self.goal.orientation.z, angleToTurn))
            if(distanceToGoal < GOAL_REACHED_TOLERANCE):
                break
            if(abs(angleToTurn) >= ANGLE_REACHED_TOLERANCE):
                self.velocity.angular.z = ANGUALR_VELOCITY_MULTIPLIER * angleToTurn;
                self.velocity.linear.x = 0
            else:
                self.velocity.linear.x = min(LINEAR_VELOCITY_MULTIPLIER * math.sqrt(dx * dx + dy * dy), MAX_LINEAR_VELOCITY);
                self.velocity.angular.z = 0
            self.velocityPublisher.publish(self.velocity);
            self.velocityPublishRate.sleep();

        # align the robot w.r.t. the orientation of the current goal
        while(not rospy.is_shutdown()):
            angleToRotate = self.angle_difference(self.goal.orientation.z, self.robotHeading);
            print('pose: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}]  goal: [x:{:0.3f}, y:{:0.3f}, th:{:0.3f}] a2r: {:0.5f}'.format(
                self.pose.position.x, self.pose.position.y, self.robotHeading, self.goal.position.x,
                self.goal.position.y, self.goal.orientation.z, angleToRotate))
            if(abs(angleToRotate) < ANGLE_REACHED_TOLERANCE):
                break;
            self.velocity.angular.z = ANGUALR_VELOCITY_MULTIPLIER * angleToRotate;
            self.velocity.linear.x = 0.0;
            self.velocityPublisher.publish(self.velocity);
            self.velocityPublishRate.sleep();

        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.velocityPublisher.publish(self.velocity);
        return;

    def run(self):
        print('{} is ready'.format(self.node_name));
        while not rospy.is_shutdown():
            if(self.state == "idle" or self.state == "reached"):
                continue;
            elif(self.state == "moving"):
                self.moveToGoal();
                self.reached_count += 1;
                self.state = "reached";
                self.reached_service(self.reached_count);


if __name__ == "__main__":
    name = sys.argv[1];
    controller = Controller(name);
    controller.run();
