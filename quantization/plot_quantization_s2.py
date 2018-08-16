"""
Plot the result of optimal quantization of the von Mises Fisher distribution
on the sphere
"""

import matplotlib.pyplot as plt

import geomstats.visualization as visualization
import quantization.optimal_quantization as quantization

from geomstats.hypersphere import Hypersphere

SPHERE2 = Hypersphere(dimension=2)
METRIC = SPHERE2.metric


def main():
    n_points = 1000
    points = SPHERE2.random_vonMisesFisher(K=10, n_samples=n_points)

    centers, n_step = quantization.optimal_quantization(
                points, n_centers=10, space='S2')

    ax = plt.subplot(111, projection="3d", aspect="equal")

    visualization.plot(centers, ax, space='S2')
    plt.show()


if __name__ == "__main__":
    main()
