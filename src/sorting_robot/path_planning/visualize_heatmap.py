import numpy as np
import matplotlib.pyplot as plt
import rospy
from sorting_robot.msg import HeatMap


def updatePlot(data):
    heatmap = np.reshape(data.heat_values, (data.rows, data.columns))
    plt.imshow(heatmap, cmap='hot', interpolation='nearest');
    plt.draw();
    plt.pause(1e-17)


def heatmapVisualizer():
    try:
        rospy.init_node('heatmap_visualizer', anonymous=True)
        plt.show()
        rospy.Subscriber('heat_map', HeatMap, updatePlot)
        rospy.spin()
    except IOError:
        print(MAP_CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config");


if __name__ == '__main__':
    heatmapVisualizer()
