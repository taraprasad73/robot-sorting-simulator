import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import rospy
from sorting_robot.msg import HeatMap
from threading import Lock

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
# if not os.path.exists(CATKIN_WORKSPACE + '/src/sorting_robot/data/heatmap'):
#     os.makedirs(CATKIN_WORKSPACE + '/src/sorting_robot/data/heatmap')
# HEATMAP_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/heatmap/{:06d}.png'
CONFIG_FILE_LOCATION = CATKIN_WORKSPACE + '/src/sorting_robot/data/{}_configuration.npy'


def initHeatmap(data, ax=None,
                cbar_kw={}, cbarlabel="", **kwargs):
    """
    Create a heatmap from a numpy array and two lists of labels.

    Parameters
    ----------
    data
        A 2D numpy array of shape (N, M).
    row_labels
        A list or array of length N with the labels for the rows.
    col_labels
        A list or array of length M with the labels for the columns.
    ax
        A `matplotlib.axes.Axes` instance to which the heatmap is plotted.  If
        not provided, use current axes or create a new one.  Optional.
    cbar_kw
        A dictionary with arguments to `matplotlib.Figure.colorbar`.  Optional.
    cbarlabel
        The label for the colorbar.  Optional.
    **kwargs
        All other arguments are forwarded to `imshow`.
    """

    im = ax.imshow(data, **kwargs)

    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(data.shape[1]))
    ax.set_yticks(np.arange(data.shape[0]))

    # ... and label them with the respective list entries.
    rowLabels = [str(i) for i in range(data.shape[0])]
    colLabels = [str(i) for i in range(data.shape[1])]

    ax.set_xticklabels(colLabels)
    ax.set_yticklabels(rowLabels)

    # Let the horizontal axes labeling appear on top.
    ax.tick_params(top=True, bottom=False,
                   labeltop=True, labelbottom=False)

    # Rotate the tick labels and set their alignment.
    plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
             rotation_mode="anchor")

    # Turn spines off and create white grid.
    for edge, spine in ax.spines.items():
        spine.set_visible(False)

    ax.set_xticks(np.arange(data.shape[1] + 1) - .5, minor=True)
    ax.set_yticks(np.arange(data.shape[0] + 1) - .5, minor=True)
    ax.grid(which="minor", color="w", linestyle='-', linewidth=3)
    ax.tick_params(which="minor", bottom=False, left=False)

    title = ax.set_title('Frame 0', y=1.08, fontsize="15", color="red")
    return im, title


def annotate_heatmap(im, data=None, valfmt="{x:.2f}",
                     textcolors=["black", "white"],
                     threshold=None, **textkw):
    """
    A function to annotate a heatmap.

    Parameters
    ----------
    im
        The AxesImage to be labeled.
    data
        Data used to annotate.  If None, the image's data is used.  Optional.
    valfmt
        The format of the annotations inside the heatmap.  This should either
        use the string format method, e.g. "$ {x:.2f}", or be a
        `matplotlib.ticker.Formatter`.  Optional.
    textcolors
        A list or array of two color specifications.  The first is used for
        values below a threshold, the second for those above.  Optional.
    threshold
        Value in data units according to which the colors from textcolors are
        applied.  If None (the default) uses the middle of the colormap as
        separation.  Optional.
    **kwargs
        All other arguments are forwarded to each call to `text` used to create
        the text labels.
    """

    if not isinstance(data, (list, np.ndarray)):
        data = im.get_array()

    # Normalize the threshold to the images color range.
    if threshold is not None:
        threshold = im.norm(threshold)
    else:
        threshold = im.norm(data.max()) / 2.

    vmax = np.max(data)
    vmin = np.min(data)
    im.set_clim(vmin, vmax)

    # Set default alignment to center, but allow it to be
    # overwritten by textkw.
    kw = dict(horizontalalignment="center",
              verticalalignment="center")
    kw.update(textkw)

    # Get the formatter in case a string is supplied
    if isinstance(valfmt, str):
        valfmt = matplotlib.ticker.StrMethodFormatter(valfmt)

    # Loop over the data and create a `Text` for each "pixel".
    # Change the text's color depending on the data.
    texts = []
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            kw.update(color=textcolors[int(im.norm(data[i, j]) > threshold)])
            text = im.axes.text(j, i, valfmt(data[i, j], None), **kw)
            texts.append(text)

    return texts


heatDataLock = Lock()
windowClosed = False


def storeHeatData(data):
    global heatData
    heatDataLock.acquire()
    heatData = np.reshape(data.heat_values, (data.rows, data.columns))
    heatDataLock.release()


def updatePlot(i, ax, im, title):
    heatDataLock.acquire()
    if heatData is None:
        return
    im.set_data(heatData)
    heatDataLock.release()
    texts = annotate_heatmap(im, valfmt="{x:.1f}", size=11, threshold=1,
                             textcolors=["black", "white"])
    title.set_text('Frame {}'.format(i))


def handle_close(evt):
    print('heatmap closed, shutting down...')
    rospy.signal_shutdown("matplotlib window was closed")


def heatmapVisualizer(mapName):
    global CONFIG_FILE_LOCATION
    CONFIG_FILE_LOCATION = CONFIG_FILE_LOCATION.format(mapName)
    try:
        mapConfiguration = np.load(CONFIG_FILE_LOCATION).item();
    except IOError:
        print(CONFIG_FILE_LOCATION +
              " doesn't exist. Run the following command to create it:\nrosrun sorting_robot generate_map_config");
    else:
        global heatData
        heatData = np.zeros((mapConfiguration['num_rows'], mapConfiguration['num_columns']))
        rospy.init_node('heatmap_visualizer', anonymous=True)
        rospy.Subscriber('heat_map', HeatMap, storeHeatData)
        fig, ax = plt.subplots()
        im, title = initHeatmap(data=heatData, ax=ax, cmap='Wistia', cbarlabel="heat value")
        heatmapAnimation = FuncAnimation(fig, updatePlot, fargs=(ax, im, title), interval=1000, repeat=True, blit=False)
        fig.canvas.mpl_connect('close_event', handle_close)
        plt.show()

        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    heatmapVisualizer('map')
