# Examples
Examples for Geomstats

These have been tested on Mac OS X 10.13.2 and linux Ubuntu 16.04.

#### Brain Connectome Analysis

The next example to try out is the brain connectome one:

```
# from the root of your unziped directory
cd brain_connectome
pip3 install -r requirements.txt
python3 spd_fmri.py
```

#### Robotics Example

The robotics example is a little bit more involved. It has its own README.md file and corresponding instructions. You will
need a C++ compilation toolchain.

```
# from the root of your unziped directory
cat robotics/README.md
```

#### Training MNIST on the Hypersphere

The deep learning on the hypersphere example requires a tensorflow patch and installing a modified version of keras.
```
# from the root of your unziped directory
cat deep_learning/README.md
```

### Riemannian Quantization

We consider here optimal quantization, an algorithm which seeks to find the best approximation, in the sense of the Wasserstein distance, of a probability distribution $\mu$ by a discrete one $\hat\mu_n$ supported by a finite number $n$ of points.

```
# from the root of your unziped directory
python3 quantization/plot_quantization_s2.py
```


Enjoy :)

# Contributors
Claire Donnat
Mikael Jorda
Alice Le Brigant
Johan Mathe
Nina Miolane
