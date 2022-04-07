# SA-Walking-Sim
Software &amp; Autonomy Team Walking Simulation. Runs a simulation of a hexapod walking up an incline.
------------------------------------------------
### To open the simulation, run the following in the first terminal:
source ~/catkin_ws/devel/setup.bash
roslaunch phantomx_gazebo phantomx_gazebo.launch

### To run the forward walking script, run the following in a second terminal:
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
python3 phantomx_gazebo/scripts/walker_demo.py
