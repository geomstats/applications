"""
Plot the result of optimal quantization of the uniform distribution
on the circle.
"""

import matplotlib.pyplot as plt
import numpy as np

import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere

CIRCLE = Hypersphere(dimension=1)


def main():
    n_points = 10000
    points = CIRCLE.random_uniform_unbounded(n_samples=n_points)

    n_centers = 5
    centers, weights, clustering, n_step = oq.optimal_quantization(
                points, n_centers, space='S1', n_repetition=20, tolerance=1e-6
                )
    theta = np.linspace(0, 2*np.pi, 100)
    circle_x = np.cos(theta)
    circle_y = np.sin(theta)
    plt.plot(circle_x, circle_y, 'k-', centers[:, 0], centers[:, 1], 'or')


if __name__ == "__main__":
    main()
