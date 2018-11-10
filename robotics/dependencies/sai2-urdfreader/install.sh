# script to install sai2-urdf

cd tinyxml2
mkdir build
cd build
cmake .. && make -j8

cd ../..
mkdir build
cd build
cmake .. && make -j8

cd ..