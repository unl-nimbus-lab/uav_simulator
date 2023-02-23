# uav_simulator
This project facilitates simulating arduCopter enabled UAVs for swarms using SITL and Gazebo. This simulator extends the steps provided by the Intelligent Quads (https://github.com/Intelligent-Quads) developers by providing a containerized solution to their work. 

# Getting started
It is recommended that this simulator be run on Ubuntu 18.04+

# Install Docker Engine
Docker Engine needs to be installed on the host machine, here is a link to the instructions for installing Docker Engine on Ubuntu:

`https://docs.docker.com/engine/install/ubuntu/`

# Clone this repository onto your machine and find the `swarm_simulator` directory:

`cd uav_simulator/swarm_simulator`

# Generate the docker-compose file
docker-compose is a powerful tool that is utilized here to launch the simulation all at once, however, there is very little control over
the "flow" of a docker-compose file. As such, a python script `generate_compose.py` is used to generate the docker-compose file based
on the type of simulation that is desired. At this point the user may specify the number of UAV's to simulate, if gazebo should be used,
and if any companion process should be used. Below is an example to generate a compose file for a single arduCopter simulation. 

`python3 generate_compose.py 1`

## generate_compose.py command-line parameters:

## Number of Vehicles
The first option specifies the number of UAVs to simulate (1-12). For instance, to generate a compose file for 5 SITL instances would be:

`python3 generate_compose.py 5`

## Include a Companion process -c
To include a ROS-based companion process use the -c flag as follows:

`python3 generate_compose.py 4 -c`

There is a default companion process specified in generate_compose.py that starts the NIMBUS clustering control. A companion process can
be used without the `-c` flag. By default, the SITL ports exposed for companion processes are 5XX3 starting at 14561 for agent 1, 14571
for agent 2 and so on.

### MAVROS
In order to connect a MAVROS node to a STIL instance use:

agent_1: `fcu_url:=udp://127.0.0.1:14561@14565`

agent_2: `fcu_url:=udp://127.0.0.1:14571@14575`

and increment the port by 10 for each subsequent agent
 
 ## Gazebo
 




