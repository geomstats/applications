You've just found the code and various examples for the NIP2018 geomstats paper!

These have been tested on Mac OS X 10.13.2 and linux Ubuntu 16.04. The deep
learning example is currently the only one which works on a GPU.

## Geomstats itself

The geomstats directory contains the code for the geomstats library. You can install it on your system by running:

```
cd geomstats
sudo pip3 install -r requirements.txt
python3 setup.py install
```

You can launch the unit tests as follow:
```
# from the root of unziped directory
cd geomstats
nose2
```

### Examples

#### Gradient Descent on Hypersphere

The easiest example to launch from the paper is the optimization on the hypersphere:

```
# from the root of your unziped directory
python3 geomstats/examples/gradient_descent_s2.py
```

#### Brain Connectome Analysis

The next example to try out is the brain connectome one:

```
# from the root of your unziped directory
pip3 install -r brain_connectome/requirements.txt
python3 brain_connectome/spd_mri.py
```

#### Robotics Example

The robotics example is a little bit more involved. It has its own README.md file and corresponding instructions. You will
need a C++ compilation toolchain.

```
# from the root of your unziped directory
cat robotics/README.md
```

#### Training MNIST on the Hypersphere

The deep learning on the hypersphere example requires a tensorflow patch and installing a modified version of keras. There is also a README.md file
```
cat deep_learning/README.md
```

Enjoy :)
