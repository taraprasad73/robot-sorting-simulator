# Sorting Robot

## Installation and setting up the environment
-----
 - Setup ROS Kinetic and catkin workspace.
    - Add ROS Kinetic API Keys
        ~~~~
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
        sudo apt-get update
        ~~~~

    - Install ROS Kinetic
        ~~~~
        sudo apt-get install ros-kinetic-ros-base
        sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
        ~~~~

    - Initialize rosdep
        ~~~~
        sudo rosdep init
        rosdep update
        ~~~~
    - Setup catkin workspace
        ~~~~
        sudo apt-get install python-pip
        pip install catkin_pkg
        echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make
        echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
        ~~~~
 - Install the STDR simulator
    ~~~~
    sudo apt-get install ros-kinetic-stdr-simulator
    ~~~~
 - If your location of catkin workspace is other than `$HOME/catkin_ws/`, let's say `$HOME/my_catkin_ws/` then set a CATKIN_WORKSPACE path variable.
    ~~~~
    echo 'export CATKIN_WORKSPACE=$HOME/my_catkin_ws/' >> $HOME/.bashrc
    ~~~~
 - Clone the sorting_robot package.
    ~~~~
    cd $HOME/catkin_ws/src/
    git clone https://github.com/taraprasad73/sorting_robot
    ~~~~
 - Install the package and its dependencies
    ~~~~
    sudo apt-get install python-tk
    pip install rospkg pyyaml empy
    pip install -r sorting_robot/requirements.txt
    cd ..
    catkin_make
    ~~~~

## Understanding the file structure of the package
-----
 - **/scripts** contains various python scripts, these files are executable and can be executed with rosrun
 - **/nodes** contains various rosnodes, these files are executable and can be executed with rosrun
 - **/src** contains the python packages, these files are used by the files in /scripts and /nodes, and shouldn't be invoked directly
 - **/msg** contains message definitions
 - **/srv** contains service definitions
 - **/launch** contains launch files
 - **/data** contains configuration files and images
 - **/stdr_data** contains files needed to launch the stdr simulator
 - **requirements.txt** contains the list of python package dependencies, can be installed through pip
 - **setup.py** is the equivalent of makefile for python, allows the scripts and nodes to access the python files from /src folder 
 - **/.circleci** contains the yaml file to execute continuous integration tests on CircleCI

### Description of various script files [TODO]
Any script file can be executed as rosrun sorting_robot name_of_script_file [command_line arguments if any]
 - generate_map_config 
 - generate_binary_map
 - generate_grid_image
 - generate_networkx_graph
 - generate_spawn_locations

### Description of various rosnode files [TODO]
Any launch file can be executed as rosrun sorting_robot name_of_launch_file [command_line arguments if any]
 - heatmap
 - path_planner
 - visualize_heatmap
 - traffic_manager

### Description of the python packages [TODO]
 - sorting_robot
   - map_generation
   - stdr_initializer
   - path_planning
   - traffic_manager
   - bfsm

## Spawn Robots on the STDR simulator
-----
 - Generate the map configuration file
    ~~~~
    rosrun sorting_robot generate_map_config
    ~~~~
    The output file will be generated in $HOME/catkin_ws/src/sorting_robot/data/ folder.

 - Launch the STDR simulator
    ~~~~
    Terminal 1: roscore
    Terminal 2: roslaunch sorting_robot stdr_server_with_map_and_gui.launch
    Terminal 3: rosrun sorting_robot generate_spawn_locations [num_of_robots_to_spawn]
    ~~~~
## Running the Sorting Robot Program
-----
 - The program is controlled by a Behavioral Finite State Machine. The following commands are used to run the low-level controller and the BFSM 
    ----
    Terminal 1: rosrun sorting_robot controller [robot_name]
    Terminal 2: rosrun sorting_robot bfsm [robot_name]
    ----
  The commands will be extended to use roslaunch for multiple robots. 
## Code Formatter Settings for Visual Studio Code
-----

### Linter standard used
 - **pep8** with E501, E703 and E402 disabled
 - **E501** - places a limit on the length of a line of code
 - **E703** - doesn't allow semicolon at the end of a statement
 - **E402** - forces module level import at top of file

### Setting pep8 in VSCode
Add the following lines to the settings.json found in File/Preferences/Settings. Don't remove any existing key value pairs present, unless its a duplicate.
~~~~
{
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": false,
    "python.linting.pep8Enabled": true,
    "python.linting.pep8Args": [
        "--ignore=E501,E703,E402"
    ],
    "python.formatting.autopep8Args": [
        "--ignore=E501,E703,E402"
    ],
}
~~~~
