# Applications

Applications for Geomstats that illustrate more involved uses of the package.

#### Brain Connectome Analysis

We consider the fMRI data from the 2014 MLSP Schizophrenia Classification challenge, consisting
of the resting-state fMRIs of 86 patients split into two balanced categories: control vs people suffering
schizophrenia.

We approach the classification task by using a SVM classifier on pairwise-similarities between brain connectomes,
represented as the SPD matrices of their regularized Laplacians. The similarities are computed with the affine-invariant
Riemannian distance on the manifold of SPD matrices.

```
# from the root of your unziped directory
pip3 install geomstats==1.7
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
pip3 install geomstats==1.7
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

#### Training a Pose Estimation Network

This example trains a pose estimation network using a SE3 Geodesic Loss function.

```
# from the root of your unziped directory
pip3 install geomstats==1.7
cat se3_pose_estimation/README.md
```


#### Impact of the curvature on the empirical Fréchet mean estimation manifolds

This application illustrates the modulation of the speed of convergence of the
empirical Fréchet mean with the curvature in spheres and hyperbolic spaces described in 
the [arXiv paper 1906.07418](https://arxiv.org/abs/1906.07418). The application is actually 
the source-code to create the figures of this paper.  

The variance of the Fréchet mean FM_n of a sample of n IID random variables of variance Var is 
decreasing more slowly in a sphere than in a Euclidean space, and more quickly in a Hyperbolic space. 
Two scripts of this application compare the speed of convergence in spheres and hyperbolic spaces to 
the classical one in Euclidean spaces by computing the  modulation factor 
alpha = Var( FM_n) / ( n * Var) for synthetic IID n-samples drawn from isotropic distributions 
on hyper-spheres (bubble distributions) of different radii. 
The last script computes the modulation factor for bootstrap n-samples drawn from a few real-world 
empirical distributions on the sphere S2 taken from the book of Fisher, Lewis and Embleton 1987.

```
# from the root of your unziped directory
cat modulation_empirical_frechet_mean/README.md
```


Enjoy :)

# Contributors

* Claire Donnat
* Benjamin Hou
* Mikael Jorda
* Alice Le Brigant
* Johan Mathe
* Nina Miolane
