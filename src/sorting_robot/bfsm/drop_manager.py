import rospy;
import numpy as np;
import threading;
from sorting_robot.msg import *;
from sorting_robot.srv import *;
from map_generation.generate_map_config import Cell, Direction, Turn, CellType;


class Drop:
    def __init__(self, x, y):
        self.pose = Pose(x, y);
        self.state = "free";
        self.possible_states = ["free", "busy"];

    def run(self):
        while not rospy.is_shutdown():
            if(self.state == "free"):
                continue;
            else:
                time.sleep(1);
                self.state = "free";


class DropManager:
    def __init__(self):
        rospy.init_node('drop_manager', anonymous=False);
        config_file = "../data/grid.npy";
        data = np.load(config_file);
        rows, cols = data.shape;
        self.drops = dict();
        self.threads = [];
        for i in range(0, len(rows)):
            for j in range(0, len(cols)):
                if(data[i][j].cellType == PARCEL_BIN):
                    self.drops[(i, j)] = Drop(i, j);
        for key in self.drops.keys():
            thread = threading.Thread(target=self.drops[key].run);
            thread.start();
            self.threads.append(thread);
        self.service = rospy.Service('/make_the_drop', Bin, self.make_drop);

    def make_drop(self, data):
        x = data.x;
        y = data.y;
        if(self.drops[(x, y)].state == "free"):
            self.drops[(x, y)].state = "busy";
            return True;
        else:
            return False;

    def close(self):
        for i in range(0, len(self.threads)):
            threads[i].join();


if __name__ == "__main__":
    manager = DropManager();
    rospy.spin();
    manager.close();
