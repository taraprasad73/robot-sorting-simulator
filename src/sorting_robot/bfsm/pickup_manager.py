import os;
import rospy;
import numpy as np;
import threading;
import random;
import time;
from sorting_robot.msg import State,Pickup;
from sorting_robot.srv import GetPickup,MakePickup;
from ..map_generation.generate_map_config import Cell,Direction,Turn,CellType;

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
	CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/map_configuration.npy'

'''
	The pickup manager deals with the pickcup related communications.
	/pickup_location - When a robot makes a call for the pickup location. The location of the queue with the 
		least amount of robots is provided. The robot is added to that queue.
	/make_pickup - The package is transferred onto the robot currently at the piclup point and the destination 
		address is provided. The robot is removed from the queue.
'''
class PickupQueue:
	def __init__(self,x,y,directions,pickup_id,queue_size):
		self.location = State(x,y,90) if(Direction.UP in directions) else State(x,y,270);
		self.pickup_id = pickup_id;
		self.queue_size = queue_size;
		self.queue = set();
		self.state = "idle";
		self.curr_robot = None;
		self.possible_states = ["idle","busy"];

	def enqueue(self,n):
		self.queue.add(n);

	def dequeue(self,n):
		self.queue.remove(n);

	def run(self):
		while not rospy.is_shutdown():
			if(self.state=="idle"):
				continue;
			else:
				time.sleep(2);
				self.dequeue(self.curr_robot);
				self.state = "idle"; 

class PickupManager:
	def __init__(self,limit=10):
		rospy.init_node('pickup_manager',anonymous=False);
		mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
		data = mapConfiguration['grid'];
		rows,cols = data.shape;
		count = 1;
		queue_size = limit;
		self.queues = [];
		self.threads = [];
		self.bins = [];
		for i in range(0,rows):
			for j in range(0,cols):
				if(data[i][j].cellType==CellType.PICKUP_QUEUE_FINISH):
					self.queues.append(PickupQueue(i,j,data[i][j].directions,count,queue_size));
					count += 1;
				elif(data[i][j].cellType==CellType.PARCEL_BIN):
					self.bins.append((i,j));
		for i in range(0,len(self.queues)):
			thread = threading.Thread(target=self.queues[i].run);
			thread.setDaemon(True);
			thread.start();
			self.threads.append(thread);
		self.select_pickup_service = rospy.Service('/pickup_location',GetPickup,self.select_pickup);
		self.make_service_service = rospy.Service('/make_pickup',MakePickup,self.make_pickup);
		print("Pickup Manager is running");

	def pick_bin(self):
		idx = random.randrange(0,len(self.bins));
		return State(self.bins[idx][0],self.bins[idx][1],0);

	def select_pickup(self,data):
		robot_name = data.robot_name.data;
		pickup_id = np.argmin([len(pickup.queue) for pickup in self.queues]);
		pickup_id = 7;
		self.queues[pickup_id].enqueue(robot_name);
		return Pickup(pickup_id,self.queues[pickup_id].location);

	def make_pickup(self,data):
		pickup_id = data.pickup_id;
		robot_name = data.robot_name.data;
		self.queues[pickup_id].curr_robot = robot_name;
		self.queues[pickup_id].state = "busy";
		return self.pick_bin();

	def close(self):
		for i in range(0,len(self.threads)):
			self.threads[i].join();

if __name__=="__main__":
	manager = PickupManager();
	rospy.spin();
	manager.close();

