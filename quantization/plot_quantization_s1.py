"""
Plot the result of optimal quantization of the uniform distribution
on the circle.
"""

import matplotlib.pyplot as plt
import geomstats.backend as gs

import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere

CIRCLE = Hypersphere(dimension=1)


def main():
    n_points = 10000
    points = CIRCLE.random_uniform_unbounded(n_samples=n_points)

    n_centers = 5
    centers, weights, clustering, n_iterations = oq.optimal_quantization(
                points, n_centers, space='S1', n_repetitions=20, tolerance=1e-6
                )
    theta = gs.linspace(0, 2*gs.pi, 100)
    circle_x = gs.cos(theta)
    circle_y = gs.sin(theta)
    plt.plot(circle_x, circle_y, 'k-', centers[:, 0], centers[:, 1], 'or')


if __name__ == "__main__":
    main()
