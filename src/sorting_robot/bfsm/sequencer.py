import os;
import rospy;
import numpy as np;
from std_msgs.msg import Int32;
from geometry_msgs.msg import Pose;
from nav_msgs.msg import Odometry;
from ..map_generation.generate_map_config import Cell, Direction, Turn, CellType;
from sorting_robot.msg import State, OccupancyMap;
from sorting_robot.srv import TrafficService,GoalService,ReachedService;
from ..utils import CoordinateSpaceManager;
from tf.transformations import euler_from_quaternion;

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
	CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'

'''
	The sequencer has three states 
	init - starting state
	moving - moving towards the goal
	reached - has reached the goal

	The sequencer communicates with the traffic manager and the controller as follows
	/subgoal service is called by the sequencer to provide the next subgoal to the controller
	/reached_subgoal is called by the controller to give an acknowledgement to the sequencer
	/traffic service is called by the sequencer to get the current traffic signal

	The process_path() function plays an important role in breaking down the sequence of steps provided
	by the path_planner into a set of subgoals
	- Every turn/intersection becomes a subgoal
	- The robot can move straight ahead upto k steps

	The follow_path() function actually communicates with the controller by providing subgoals and receiving 
	acknoledgements for the same. When at intersections, it communicates with the traffic manager. This function
	is called by the BFSM.

'''

class Sequencer:
	def __init__(self, name):
		self.init_map();
		self.pose = State();
		self.state = "init";
		self.possible_states = ["init", "moving", "reached"];
		self.name = name;
		self.prev_reached = 0;
		self.csm = CoordinateSpaceManager();
		#self.publisher = rospy.Publisher('/' + name + '/subgoal', Pose, queue_size=10);
		#self.reached_subscriber = rospy.Subscriber('/' + name + '/reached_subgoal', Int32, self.reached_callback);
		self.pose_subscriber = rospy.Subscriber('/' + name + '/odom', Odometry, self.odom_callback);
		self.map_subscriber = rospy.Subscriber('/occupancy_map', OccupancyMap, self.map_callback);
		self.reached_service = rospy.Service('/'+name+'/reached_subgoal',ReachedService,self.received_ack);
		self.goal_service = rospy.ServiceProxy('/'+name+'/subgoal',GoalService);
		self.traffic_service = rospy.ServiceProxy('/traffic', TrafficService);

	def init_map(self):
		mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
		self.data = mapConfiguration['grid'];
		rows, cols = self.data.shape;
		occupancy_map = [];
		for i in range(0, rows):
			occupancy_map.append([False] * cols);
		self.occupancy_map = occupancy_map;

	'''
	def reached_callback(self, data):
		if(data.data != self.prev_reached):
			self.state = "reached";
			self.prev_reached = data.data;
			print("Reached goal");
			print(self.prev_reached);
	'''

	def received_ack(self,data):
		reached_goal = data.count;
		print("Reached goal");
		print(self.prev_reached);
		self.state = "reached";
		self.prev_reached = reached_goal;
		return 1;
	
	def odom_callback(self, data):
		pose = data.pose.pose;
		theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2];
		x, y, theta = self.csm.convertPointToState((pose.position.x, pose.position.y, theta));
		self.pose.row = x;
		self.pose.col = y;
		self.pose.direction = theta;

	def map_callback(self, data):
		rows = data.rows;
		cols = data.columns;
		new_map = [];
		k = 0;
		for i in range(0, rows):
			temp = [];
			for j in range(0, cols):
				temp.append(data.occupancy_values[k]);
				k += 1;
			new_map.append(temp);
		self.occupancy_map = new_map;

	def have_to_turn(self, src, dest):
		if(src.direction == dest.direction):
			return False;
		else:
			return True;

	def isSameCell(self,prev,curr):
		if(prev.row==curr.row and prev.col==curr.col):
			return True;
		return False;

	def isSameState(self,prev,curr):
		if(prev.row==curr.row and prev.col==curr.col and prev.direction==curr.direction):
			return True;
		return False;

	def isIntersection(self, x, y):
		if(self.data[x][y].cellType == CellType.STREET_STREET_INTERSECTION):
			return True;
		elif(self.data[x][y].cellType == CellType.HIGHWAY_HIGHWAY_INTERSECTION):
			return True;
		if(self.data[x][y].cellType == CellType.HIGHWAY_STREET_INTERSECTION):
			return True;
		return False;

	def process_path(self, path, k=5):
		if(len(path) <= 1):
			return path;
		processed_path = [];
		traffic_stops = [];
		prev = path[0];
		count = 0;
		for i in range(1,len(path)):
			'''
			if(self.have_to_turn(prev, path[i]) == True):
				processed_path.append(path[i]);
				count = 0;
			'''
			if(self.isIntersection(path[i].row,path[i].col)):
				if(self.isSameCell(prev,path[i])==True):
					processed_path.append(path[i]);
					traffic_stops.append(None);
					count = 0;
				elif(self.isIntersection(prev.row,prev.col)==False):
					processed_path.append(prev);
					traffic_stops.append(path[i]);
					count = 0;
				else:
					count += 1;
			elif(count==k):
				processed_path.append(path[i]);
				traffic_stops.append(None);
				count = 0;
			elif(i==len(path)-1):
				processed_path.append(path[i]);
				traffic_stops.append(None);
			else:
				count += 1;
			prev = path[i];

		return processed_path,traffic_stops;

	def follow_path(self, path):
		path,stops = self.process_path(path, 5);
		prev = self.pose;
		for i in range(0, len(path)):
			pose = Pose();
			print(path[i]);
			world = self.csm.getWorldCoordinateWithDirection((path[i].row, path[i].col, path[i].direction));
			pose.position.x = world[0];
			pose.position.y = world[1];
			pose.orientation.z = world[2];
			if(stops[i]==None):
				self.goal_service(pose);
				self.state = "moving";
			else:
				while(True):
					direction = self.traffic_service(stops[i]);
					if(prev.orientation==0 and direction.right==True):
						break;
					elif(prev.orientation==180 and direction.left==True):
						break;
					elif(prev.orientation==90 and direction.up==True):
						break;
					elif(prev.orientation==270 and direction.down==True):
						break;
					time.sleep(1);
				self.goal_service(pose);
				self.state = "moving";
			'''
			if(self.have_to_turn(prev,path[i])==True):
				self.goal_service(pose);
				self.state = "moving";
			else:
				x = path[i].row;
				y = path[i].col;
				while(self.occupancy_map[x][y]==True):
					continue;
				if(isIntersection(x,y)==True):
					while(True):
						direction = self.traffic_service(State(x,y,0));
						if(prev.orientation==0 and direction.right==True):
							break;
						elif(prev.orientation==180 and direction.left==True):
							break;
						elif(prev.orientation==90 and direction.up==True):
							break;
						elif(prev.orientation==270 and direction.down==True):
							break;
						else:
							continue;
				self.goal_service(pose);
				self.state = "moving";
			'''
			print("Published goal");
			while(self.state=="moving"):
				if(rospy.is_shutdown()==True):
					break;
			prev = path[i];
			if(rospy.is_shutdown()==True):
				break;
		self.state = "init";
		return;
