# uav_simulator
This project facilitates simulating arduCopter enabled UAVs for swarms using SITL and Gazebo. This simulator extends the steps provided by the Intelligent Quads (https://github.com/Intelligent-Quads) developers by providing a containerized solution to their work. 

# Getting started
It is recommended that this simulator be run on Ubuntu 18.04+

## Install docker engine
Docker Engine needs to be installed on the host machine, here is a link to the instructions for installing Docker Engine on Ubuntu:
`https://docs.docker.com/engine/install/ubuntu/`

## Clone this repository onto your machine and find the `swarm_simulator` directory:
`cd uav_simulator/swarm_simulator`

## Generate the docker-compose file
docker-compose is a powerful tool that is utilized here to launch the simulation all at once, however, there is very little control over
the "flow" of a docker-compose file. As such, a python script `generate_compose.py` is used to generate the docker-compose file based
on the type of simulation that is desired. At this point the user may specify the number of UAV's to simulate, if gazebo should be used,
and if any companion process should be used. 

`python3 generate_compose.py 1`

### There are a few command line options:
The first option specifies the number of UAVs to simulate (1-12). For instance, to generate a simulation for 

