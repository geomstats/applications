# sai2-model

This is SAI2 robot model library for robot kinematics and dynamics.
It uses [RBDL](https://rbdl.bitbucket.io/) and adds function to facilitate the implementation of the whole body control framework

## Dependencies
sai2-model depends on eigen3 amd sai2-urdfreader.
You can get sai2-urdfreader [there](https://github.com/manips-sai-org/sai2-urdfreader)

## Build instructions
You can use the provided install script for automatic install
```
sh install.sh
```
Or you can install manually :
 * First go to the rbdl folder and make rbdl
 ```
cd rbdl
mkdir build
cd build
cmake .. && make -j8
cd ../..
```
 * Then you can make sai2-model from the base directory
```
mkdir build
cd build
cmake .. && make -j8
```

## run the examples
Go to the build/examples/one_of_the_examples folder and run the example. For example 1 :
```
cd build/examples/01-create_model_from_file
./01-create_model_from_file
```
