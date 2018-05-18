This example illustrates training of mnist on an hypersphere manifold.

## Installation

The installation of this example is currently quite involved. You will need to:

- Update tensorflow from a patch file in this repository (re-compiling would be a nightmare - this solution is a hack but it is the least painful path for now.). You will also need a fairly recent version of tensorflow (1.8 at least). BEWARE! This *will* alter your version of tensorflow - and might break it. You need to
- Update keras (the following steps are just if you've never installed keras - you might want to uninstall it before re-installing this version):
```
cd keras/
python3 setup.py
```
- Make sure you have geomstats installed (pip3 install geomstats or install it following the steps in the main README.md file)

## Running

```
python3 mnist_hypersphere.py
```
