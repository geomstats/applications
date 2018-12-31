# Applications
Applications for Geomstats

These have been tested on Mac OS X 10.13.2 and linux Ubuntu 16.04.

#### Brain Connectome Analysis

We consider the fMRI data from the 2014 MLSP Schizophrenia Classification challenge, consisting
of the resting-state fMRIs of 86 patients split into two balanced categories: control vs people suffering
schizophrenia.

We approach the classification task by using a SVM classifier on pairwise-similarities between brain connectomes,
represented as the SPD matrices of their regularized Laplacians. The similarities are computed with the affine-invariant
Riemannian distance on the manifold of SPD matrices.

```
# from the root of your unziped directory
cd brain_connectome
pip3 install -r requirements.txt
python3 spd_fmri.py
```

#### Robotics Application

We consider a robot arm in this application. In robotics, it is common to control a manipulator in Cartesian space rather
than configuration space. We generate a geodesic on SO(3) between the initial orientation of the robot arm and its
desired final orientation. We use the generated trajectory as an input to the robot controller.

The robotics application is a little bit more involved. It has its own README.md file and corresponding instructions. You will
need a C++ compilation toolchain.

```
# from the root of your unziped directory
cat robotics/README.md
```

#### Training MNIST on the Hypersphere

We consider the training of MNIST with weights constrained on the Hypersphere. The optimization step has been modified in keras
such that the stochastic gradient descent is done on the manifold through the Exponential Map.

The deep learning application requires a tensorflow patch and installing a modified version of keras.
```
# from the root of your unziped directory
cat deep_learning/README.md
```

### Training a Pose Estimation Network

This example trains a pose estimation network using a SE3 Geodesic Loss function.

```
# from the root of your unziped directory
cat se3_pose_estimation/README.md
```

Enjoy :)

# Contributors

* Claire Donnat
* Benjamin Hou
* Mikael Jorda
* Alice Le Brigant
* Johan Mathe
* Nina Miolane
