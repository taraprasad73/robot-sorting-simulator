import os
import re
import xmltodict
import rospy

HOME_DIR = os.environ['HOME']
CATKIN_WORKSPACE = HOME_DIR + '/catkin_ws/'
if os.environ.get('CATKIN_WORKSPACE'):
    CATKIN_WORKSPACE = os.environ['CATKIN_WORKSPACE']
CONTROLLERS_LAUNCH_FILE = CATKIN_WORKSPACE + '/src/sorting_robot/launch/controllers.launch'
BFSMS_LAUNCH_FILE = CATKIN_WORKSPACE + '/src/sorting_robot/launch/bfsms.launch'


def get_available_robots():
    newRobots = set()
    topics = rospy.get_published_topics()
    for topic in topics:
        topic_name, topic_type = topic;
        match = re.search("/.*/odom", topic_name)
        if(match):
            check = re.search("/.*/", topic_name)
            robotName = check.group()[1:len(check.group()) - 1]
            newRobots.add(robotName)
    return list(newRobots)


def create_controller_launch_xml():
    xmlDict = {}
    xmlDict['launch'] = {}
    robots = get_available_robots()
    nodes = []
    for robot in robots:
        node = {'@pkg': "sorting_robot",
                '@type': "controller",
                '@output': "log",
                '@cwd': "node"}
        node['@name'] = robot + '_controller'
        node['@args'] = robot
        nodes.append(node)
    xmlDict['launch']['node'] = nodes
    return xmlDict


def create_bfsm_launch_xml():
    xmlDict = {}
    xmlDict['launch'] = {}
    robots = get_available_robots()
    nodes = []
    for robot in robots:
        node = {'@pkg': "sorting_robot",
                '@type': "bfsm",
                '@output': "log",
                '@cwd': "node"}
        node['@name'] = robot + '_bfsm'
        node['@args'] = robot
        nodes.append(node)
    xmlDict['launch']['node'] = nodes
    return xmlDict


def write_xml_to_file(xml, file):
    xml_str = xmltodict.unparse(xml, pretty=True)
    with open(file, 'w') as fd:
        fd.write(xml_str)


def generate_launch_files():
    controller_xml = create_controller_launch_xml()
    bfsm_xml = create_bfsm_launch_xml()
    write_xml_to_file(controller_xml, CONTROLLERS_LAUNCH_FILE)
    write_xml_to_file(bfsm_xml, BFSMS_LAUNCH_FILE)