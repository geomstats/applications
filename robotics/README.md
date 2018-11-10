# Geomstats Robotics Example

This is a simulation and controller of a robot to use with geomstats example robot_so3.py.
It uses [gazebo](http://gazebosim.org/) for the simulation and Stanford Robotics Lab software for the control part.

The example runs a simulation and controller of a KUKA IIWA14 robot. The controller is an operational space position and orientation controller at the end effector of the robot with a nullspace pose task that keeps the elbow up.

The controller reads the desired position and orientation from a redis server, populated by geomstat examples

## Dependencies
This application depends on:

* c++ compiler, cmake [brew, apt-get]
* geomstats: pip3 install geomstats
* [Gazebo](http://gazebosim.org/) versions 7 to 9
* sai2-model: Install from dependencies/sai2-model
* sai2-urdfreader: Install from dependencies/sai2-urdfreader
* redis: Redis server [brew install redis, apt-get install redis]
* hiredis: Redis minimalist client [brew install hiredis, apt-get install ]
* eigen3: Linear algebra [brew, apt-get]
* redis for python3 : pip3 install redis

## Build instructions
```
mkdir build
cd build
cmake .. && make -j2
```

## run
* First, make sure the redis server is running. To run it type in a terminal
```
redis-server
```
* Second, run the simulation and the controller using the provided script
```
./run_simulation.sh
```
* Third, run the python script robot_so3.py
```
python3 robot_so3.py
```
