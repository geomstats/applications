"""
Plot the result of optimal quantization of the uniform distribution
on the circle.
"""

import geomstats.visualization as visualization
import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere

CIRCLE = Hypersphere(dimension=1)
METRIC = CIRCLE.metric
N_POINTS = 1000
N_CENTERS = 5
N_REPETITIONS = 20
TOLERANCE = 1e-6


def main():
    points = CIRCLE.random_uniform(n_samples=N_POINTS, bound=None)

    centers, weights, clustering, n_iterations = oq.optimal_quantization(
                points=points, metric=METRIC, n_centers=N_CENTERS,
                n_repetitions=N_REPETITIONS, tolerance=TOLERANCE
                )
    visualization.plot(centers, space='S1', color='red')


if __name__ == "__main__":
    main()
