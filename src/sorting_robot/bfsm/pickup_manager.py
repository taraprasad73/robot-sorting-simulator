import rospy;
import numpy as np;
import threading;
import random;
from sorting_robot.msg import Pickup,ReadyToPickup;
from sorting_robot.srv import GetPickup;
from map_generation.generate_map_config import Cell,Direction,Turn,CellType;

class Pickup:
	def __init__(self,x,y,pickup_id,queue_size):
		self.pose = Pose(x,y);
		self.pickup_id = pickup_id;
		self.queue_size = queue_size;
		self.queue = set();
		self.state = "idle";
		self.curr_robot = None;
		self.possible_states = ["idle","busy"];

	def enqueue(self,n):
		self.queue.add(n);

	def dequeue(self):
		self.queue.remove(n);

	def run(self):
		while not rospy.is_shutdown():
			if(self.queues.state=="idle"):
				continue;
			else:
				time.sleep(2);
				self.queue.dequeue(self.curr_robot);
				self.state = "idle"; 

class PickupManager:
	def __init__(self,n,limit):
		rospy.init_node('pickup_manager',anonymous=False);
		config_file = "../data/grid.npy";
		data = np.load(config_file);
		rows,cols = data.shape;
		count = 1;
		queue_size = 10;
		self.queues = [];
		self.threads = [];
		self.bins = [];
		for i in range(0,len(rows)):
			for j in range(0,len(cols)):
				if(data[i][j].isPickupEntry==True):
					self.queues.append(Pickup(i,j,count,queue_size));
					count += 1;
				elif(data[i][j].cellType==CellType.PARCEL_BIN):
					self.bins.append((i,j));
		for i in range(0,len(self.queues)):
			thread = threading.Thread(target=self.queues[i].run);
			thread.start();
			self.threads.append(thread);
		self.select_pickup_service = rospy.Service('/pickup_location',GetPickup,self.select_pickup);
		self.make_service_service = rospy.Service('/make_pickup',MakePickup,self.make_pickup);

	def pick_bin(self):
		idx = random.randrange(0,len(self.bins));
		return State(self.bins[idx][0],self.bins[idx][1],0);

	def select_pickup(self,data):
		name = data.robot_name;
		pickup_id = np.argmin([q.size() for q in self.queues]);
		self.queues[pickup_id].enqueue(name);
		return Pickup(pickup_id,self.queues[pickup_id].location);

	def make_pickup(self,data):
		pickup_id = data.pickup_id;
		robot_name = data.robot_name;
		self.queues[pickup_id].state = "busy";
		self.queues[pickup_id].curr_robot = robot_name;
		return self.pick_bin();

	def close(self):
		for i in range(0,len(self.threads)):
			threads[i].join();

if __name__=="__main__":
	manager = PickupManager();
	rospy.spin();
	manager.close();

