This example illustrates training of MNIST on an hypersphere manifold.

## Installation

The installation of this example is currently quite involved. You will need to:

- Update tensorflow from a patch file in this repository (re-compiling would be a nightmare - this solution is a hack but it is the least painful path for now.). You will also need a tensorflow version 1.8.

BEWARE! This *will* alter your version of tensorflow - and might break it. You need to overwrite the variables.py file in the right place.
For instance:

```
find /usr/local/ -iname "variables.py"
/usr/local/lib/python3.5/dist-packages/tensorflow/python/ops/variables.py
# Backup your file:
cp /usr/local/lib/python3.5/dist-packages/tensorflow/python/ops/variables.py backup_variables.py
# Copy the new file:
cp tf_patch/variables /usr/local/lib/python3.5/dist-packages/tensorflow/python/ops/variables.py
```

- Update keras (the following steps are just if you've never installed keras - you might want to uninstall it before re-installing this version):
```
cd keras/
sudo python3 setup.py install
```
- Make sure you have geomstats installed (pip3 install geomstats or install it following the steps in the main README.md file)

## Running MNIST on Hypersphere:

```
./run.sh
```
