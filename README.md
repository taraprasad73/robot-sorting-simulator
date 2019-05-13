# Sorting Robot

## Setting up the environment
 - Setup ROS Kinetic and catkin workspace.
    ~~~~
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-ros-base
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make
    echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
    ~~~~
 - Install the STDR simulator, refer http://wiki.ros.org/stdr_simulator
 - If your location of catkin workspace is other than $HOME/catkin_ws/ then set a CATKIN_WORKSPACE path variable.
    ~~~~
    echo 'export CATKIN_WORKSPACE=$HOME/my_catkin_ws/' >> $HOME/.bashrc
    ~~~~
 - Setup the sorting_robot package.
    ~~~~
    cd $HOME/catkin_ws/src/
    git clone https://github.com/taraprasad73/sorting_robot
    pip install -r sorting_robot/requirements.txt
    cd ..
    catkin_make
    ~~~~

## Understanding the file structure of the package
 - **/scripts** contains various python scripts, these files are executable and can be executed with rosrun
 - **/nodes** contains various rosnodes, these files are executable and can be executed with rosrun
 - **/src** contains the python packages, these files are used by the files in /scripts and /nodes, and shouldn't be invoked directly
 - **/msg** contains message definitions
 - **/srv** contains service definitions
 - **/launch** contains launch files
 - **/data** contains temporary files, this folder is added to .gitignore and hence is not tracked by git
 - **/stdr_data** contains files needed to launch the stdr simulator
 - **requirements.txt** contains the list of python package dependencies, can be installed through pip
 - **setup.py** is the equivalent of makefile for python, allows the scripts and nodes to access the python files from /src folder 

## Description of various script files [TODO]
Any script file can be executed as rosrun sorting_robot name_of_script_file [command_line arguments if any]
 - generate_map_config 
 - generate_binary_map
 - generate_grid_image
 - generate_networkx_graph
 - generate_spawn_locations

## Description of various rosnode files [TODO]
Any launch file can be executed as rosrun sorting_robot name_of_launch_file [command_line arguments if any]
 - heatmap

## Description of the python packages [TODO]
 - sorting_robot
   - map_generation
   - stdr_initializer
   - path_planning
   - traffic_controller
   - bfsm

## Spawn Robots on the STDR simulator
### Generate the map configuration file
~~~~
rosrun sorting_robot generate_map_config

The output file will be generated in $HOME/catkin_ws/src/sorting_robot/data/ folder.
~~~~

### Generate the spawn locations and execute the script
~~~~
rosrun sorting_robot generate_spawn_locations.py [num_of_robots_to_spawn]
chmod +x $HOME/catkin_ws/src/sorting_robot/data/spawn_robots.sh
~~~~

### Launch the STDR simulator
~~~~
Terminal 1: roscore
Terminal 2: roslaunch sorting_robot stdr_server_with_map_and_gui.launch
Terminal 3: bash $HOME/catkin_ws/src/sorting_robot/data/spawn_robots.sh
~~~~

## Code Formatter Settings for Visual Studio Code

### Linter standard used
 - **pep8** with E501 and E703 disabled
 - **E501** - places a limit on the length of a line of code
 - **E703** - doesn't allow semicolon at the end of a statement

### Setting pep8 in VSCode
Add the following lines to the settings.json found in File/Preferences/Settings. Don't remove any existing key value pairs present, unless its a duplicate.
~~~~
{
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": false,
    "python.linting.pep8Enabled": true,
    "python.linting.pep8Args": [
        "--ignore=E501,E703"
    ],
    "python.formatting.autopep8Args": [
        "--ignore=E501,E703"
    ],
}
~~~~
