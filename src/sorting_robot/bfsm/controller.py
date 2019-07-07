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

The move() function which actually moves the robot to its goal is described below
kv - Constant for linear velocity of the robot
kw - Constant for angular velocity of the robot
dg - Distance of the robot from the goal
dx - Distance of the robot from the goal along x-axis
dy - Distance of the robot from the goal along y-axis
a2g - Angle to goal
diff - Difference between the robot orientation and the angle to goal
The first while loop is for linear movement while the next is for pure angular motion.
'''


class Controller:
    def __init__(self, name):
        rospy.init_node(name + '_controller', anonymous=False);
        self.pose = Pose();
        self.goal = Pose();
        self.velocity = Twist();
        self.theta = 0;
        self.reached_count = 0;
        self.name = name;
        self.state = "idle";
        self.possible_states = ["idle", "moving", "reached"];
        self.pose_subscriber = rospy.Subscriber('/' + name + '/odom', Odometry, self.odom_callback);
        # self.laser_subscriber = rospy.Subscriber('/' + name + '/laser_0', LaserScan, self.laser_callback);
        self.goal_service = rospy.Service('/' + name + '/subgoal', GoalService, self.receive_goal);
        self.reached_service = rospy.ServiceProxy('/' + name + '/reached_subgoal', ReachedService);
        self.publisher = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=10);
        self.rate = rospy.Rate(1);

    def odom_callback(self, data):
        self.pose = data.pose.pose;
        self.theta = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,
                                            self.pose.orientation.z, self.pose.orientation.w])[2];
        # print('odom callback: pose={} theta={}'.format(self.pose.position, self.theta))

    def laser_callback(self, data):
        self.laser_scan = list(data.ranges);

    def receive_goal(self, data):
        self.goal = data.goal
        print("Received goal from sequencer");
        print(self.goal);
        self.state = "moving";
        return 1;

    def angle_difference(self, angle1, angle2):
        if(abs(angle1 - angle2) <= 3.14):
            return angle1 - angle2;
        elif((angle1 - angle2) > 3.14):
            return -(6.28 - (angle1 - angle2));
        else:
            return 6.28 + (angle1 - angle2);

    def new_goal(self, data):
        dist = math.sqrt(math.pow(data.position.x - self.goal.position.x, 2) +
                         math.pow(data.position.y - self.goal.position.y, 2));
        diff = self.angle_difference(data.orientation.z, self.goal.orientation.z);
        if(dist <= 0.1 and abs(diff) <= 0.01):
            return False;
        else:
            return True;

    def move(self):
        kv = 2;
        kw = 2;
        dg = 5;
        while(not rospy.is_shutdown() and dg > 0.05):
            dx = self.goal.position.x - self.pose.position.x;
            dy = self.goal.position.y - self.pose.position.y;
            print('pose: ({}, {}) goal: ({}, {})'.format(self.pose.position.x, self.pose.position.y, self.goal.position.x, self.goal.position.y))
            a2g = np.arctan2(dy, dx);
            diff = np.arctan2(math.sin(a2g - self.theta), math.cos(a2g - self.theta));
            dg = math.sqrt(dx * dx + dy * dy);
            if(abs(diff) > 0.02):
                self.velocity.angular.z = kw * diff;
                self.velocity.linear.x = 0.0;
            else:
                self.velocity.linear.x = kv * math.sqrt(dx * dx + dy * dy);
                self.velocity.angular.z = 0.0;
            self.publisher.publish(self.velocity);
        while(not rospy.is_shutdown()):
            diff = self.angle_difference(self.goal.orientation.z, self.theta);
            if(abs(diff) <= 0.1):
                break;
            self.velocity.angular.z = kw * diff;
            self.velocity.linear.x = 0.0;
            self.publisher.publish(self.velocity);
        self.rate.sleep();
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.publisher.publish(self.velocity);
        return;

    def run(self):
        print('Controller ready');
        while not rospy.is_shutdown():
            if(self.state == "idle" or self.state == "reached"):
                continue;
            elif(self.state == "moving"):
                self.move();
                self.reached_count += 1;
                self.state = "reached";
                self.reached_service(self.reached_count);


if __name__ == "__main__":
    name = sys.argv[1];
    controller = Controller(name);
    controller.run();
