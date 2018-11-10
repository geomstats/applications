# sai2-urdfreader
This is a standalone port of the ROS urdf reader for use with the SAI2 libraries.

## Build instructions 
You can use the provided install script for automatic install
```
sh install.sh
```
Or you can install manually :
 * First go to the tinyxml2 folder and make tinyxml2
 ```
cd tinyxml2
mkdir build
cd build
cmake .. && make -j8
cd ../..
```
 * Then you can make sai2-urdf from the base directory
```
mkdir build
cd build
cmake .. && make -j8
