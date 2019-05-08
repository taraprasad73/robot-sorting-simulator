# Sorting Robot

## Setup the environment
~~~~
pip install -r requirements.txt
echo 'export PYTHONPATH="${PYTHONPATH}:/path/to/project"' >> .bashrc
Setup ROS Kinetic and catkin workspace.
Install the STDR simulator, refer http://wiki.ros.org/stdr_simulator
~~~~

## ROS Package Creation
~~~~
catkin_create_pkg sorting_robot geometry_msgs nav_msgs rospy std_msgs message_generation message_runtime
~~~~

## Spawn Robots on the STDR simulator
Copy the stdr_launcher into catkin_ws/src and recompile. Then run the launcher to open stdr simulator.
~~~~
cp -a stdr_launcher/. ~/catkin_ws/src/stdr_launcher
cd ~/catkin_ws
catkin_make
roslaunch stdr_launcher server_with_map_and_gui.launch
~~~~

### Generate the configuration file of the map
~~~~
python map_generation/generate_map_config.py
[optional] python map_generation/generate_colored_map.py

The output files will be present in the /data folder.
~~~~

### Generate the spawn locations and execute the script
~~~~
python user_interface/generate_spawn_locations.py 5
chmod +x data/spawn_robots.sh
./data/spawn_robots.sh
~~~~