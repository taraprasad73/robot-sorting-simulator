import os;
import rospy;
import numpy as np;
from std_msgs.msg import Int32;
from geometry_msgs.msg import Pose;
from nav_msgs.msg import Odometry;
from ..map_generation.generate_map_config import Cell,Direction,Turn,CellType;
from sorting_robot.msg import State,OccupancyMap;
from sorting_robot.srv import TrafficService;
from ..utils import CoordinateSpaceManager;
from tf.transformations import euler_from_quaternion;

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ['CATKIN_WORKSPACE']:
	CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'

class Sequencer:
	def __init__(self,name):
		self.init_map();
		self.pose = State();
		self.state = "init";
		self.possible_states = ["init","moving","reached"];
		self.name = name;
		self.prev_reached = 0;
		self.csm = CoordinateSpaceManager();
		self.publisher = rospy.Publisher('/'+name+'/subgoal',Pose,queue_size=10);
		self.reached_subscriber = rospy.Subscriber('/'+name+'/reached_subgoal',Int32,self.reached_callback);
		self.pose_subscriber = rospy.Subscriber('/'+name+'/odom',Odometry,self.odom_callback);
		self.map_subscriber = rospy.Subscriber('/occupancy_map', OccupancyMap, self.map_callback);
		self.traffic_service = rospy.ServiceProxy('/traffic',TrafficService);

	def init_map(self):
		mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
		self.data = mapConfiguration['grid'];
		rows, cols = self.data.shape;
		occupancy_map = [];
		for i in range(0,rows):
			occupancy_map.append([False]*cols);
		self.occupancy_map = occupancy_map;

	def reached_callback(self,data):
		if(data!=self.prev_reached):
			self.state = "reached";
			self.prev_reached = data;
			print("Reached goal");
			print(self.prev_reached);

	def odom_callback(self,data):
		pose = data.pose.pose;
		theta = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])[2];
		x,y,theta = self.csm.convertPointToState((pose.position.x,pose.position.y,theta));
		self.pose.row = x;
		self.pose.col = y;
		self.pose.direction = theta; 

	def map_callback(self):
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
		self.occupancy_map = new_map;

	def have_to_turn(self,src,dest):
		if(src.direction==dest.direction):
			return False;
		else:
			return True;

	def process_path(self,path,k=3):
		if(len(path)<=1):
			return path;
		processed_path = [path[0]];
		prev = path[0];
		count = 0;
		for i in range(0,len(path)):
			if(self.have_to_turn(prev,path[i])==True):
				processed_path.append(prev);
				processed_path.append(path[i]);
				count = 0;
			elif(count==k):
				processed_path.append(path[i]);
				count = 0;
			elif(i==len(path)-1):
				processed_path.append(path[i]);
			else:
				count += 1;
			prev = path[i];

		return processed_path;
	
	def isIntersection(self,x,y):
		if(self.data[x][y].cellType==STREET_STREET_INTERSECTION):
			return True;
		elif(self.data[x][y].cellType==HIGHWAY_HIGHWAY_INTERSECTION):
			return True;
		if(self.data[x][y].cellType==HIGHWAY_STREET_INTERSECTION):
			return True;
		return False;

	def follow_path(self,path):
		#path = [(46,18,90),(45,18,90),(44,18,90),(43,18,90),(42,18,90),(41,18,90),(41,18,180),(41,17,180),(41,16,180)];
		#path = [State(p[0],p[1],p[2]) for p in path];
		path = self.process_path(path);
		prev = self.pose;
		for i in range(0,len(path)):
			pose  = Pose();
			print(path[i]);
			world = self.csm.getWorldCoordinateWithDirection((path[i].row,path[i].col,path[i].direction));
			pose.position.x = world[0];
			pose.position.y = world[1];
			pose.orientation.z = world[2];
			'''
			if(self.have_to_turn(prev,path[i])==True):
				self.publisher.publish(pose);
				self.state = "moving";
			else:
				x = path[i].row;
				y = path[i].col;
				while(True):
					if(self.occupancy_map[x][y]==False):
						break;
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
					self.publisher.publish(pose);
					self.state = "moving";
				else:
					self.publisher.publish(pose);
					self.state = "moving";
			'''
			self.publisher.publish(pose);
			self.state = "moving";
			print("Published goal");
			while(self.state=="moving"):
				self.publisher.publish(pose);
			prev = path[i];
		self.state = "init";
		return;


