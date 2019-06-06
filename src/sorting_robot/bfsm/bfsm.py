import rospy;
import numpy as np;
from sorting_robot.msg import State,Pickup,ReadyToPickup;
from sorting_robot.srv import Path,PathInPickup,PathToBin,GetPickup;
from nav_msgs.msg import Odometry;
from geometry_msgs.msg import Pose;
from sequencer import *;

class BFSM:
	def __init__(self,name):
		rospy.init_node(name+'_bfsm',anonymous=False);
		self.state = "select_pickup";
		self.possible_states = ["go_to_pickup","select_pickup","making_pickup","make_the_drop","go_to_charge","select_charge","charging"];
		self.name = name;
		self.pose = Pose();
		self.pickup_location = Pose();
		self.drop_location = Pose();
		self.parcel_id = None;
		self.sequencer = Sequencer(name);
		self.pose_subscriber = rospy.Subscriber('/'+name+'/odom',Odometry,odom_callback);
		self.path_service = rospy.ServiceProxy('/path',Path);
		self.bin_service = rospy.ServiceProxy('/path_to_bin',PathToBin);
		self.pickup_service = rospy.ServiceProxy('/pickup_location',GetPickup);
		self.ready_to_pickup =rospy.Publisher('/ready_to_pickup',ReadyToPickup);

	def odom_callback(self,data):
		self.pose = data.pose.pose;
		
	def run(self):
		path = [];
		self.state = "go_to_pickup";
		self.sequencer.follow_path(path);
		print('Completed BFSM..');
		'''
		while not rospy.is_shutdown():
			if(self.state=="go_to_pickup"):
				path = self.path_service(self.pose,self.pickup_location);
				self.sequencer.follow_path(path.path);
				self.state = "making_pickup";
			elif(self.state=="select_pickup"):
				self.pickup_location = self.pickup_service();
				self.state = "go_to_pickup";
			elif(self.state=="make_the_pickup"):
				self.ready_to_pickup.publish(ReadytoPickup(self.name,self.pickup_location));
				time.sleep(1);
				self.get_bin_location();
				self.state = "go_to_bin";
			elif(self.state=="go_to_bin"):
				path = self.bin_service(self.pose,self.bin_location);
				self.sequencer.follow_path(path.path);
				self.state = "make_the_drop";
			elif(self.state=="make_the_drop"):
				time.sleep(1);
				self.state = "select_pickup";
		'''

if __name__=="__main__":
	name = sys.argv[1];
	bfsm = BFSM(name);
	bfsm.run();