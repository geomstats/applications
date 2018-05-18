# script to install sai2-model

cd rbdl
mkdir build
cd build
cmake .. && make -j8

cd ../..
mkdir build
cd build
cmake .. && make -j8

cd ..