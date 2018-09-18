"""
Plot the result of optimal quantization of the von Mises Fisher distribution
on the sphere
"""

import matplotlib.pyplot as plt

import geomstats.backend as gs
import geomstats.visualization as visualization
import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere

SPHERE2 = Hypersphere(dimension=2)
METRIC = SPHERE2.metric
N_POINTS = 1000
N_CENTERS = 5
N_REPETITIONS = 20
KAPPA = 10


def main():

    points = SPHERE2.random_von_mises_fisher(kappa=KAPPA, n_samples=N_POINTS)

    centers, weights, clusters, n_steps = oq.optimal_quantization(
                points=points, metric=METRIC, n_centers=N_CENTERS,
                n_repetitions=N_REPETITIONS
                )

    plt.figure(0)
    ax = plt.subplot(111, projection="3d", aspect="equal")
    visualization.plot(centers, ax, space='S2', c='r')
    plt.show()

    plt.figure(1)
    ax = plt.subplot(111, projection="3d", aspect="equal")
    color = gs.random.rand(N_CENTERS, 3)
    for i in range(N_CENTERS):
        sphere = visualization.Sphere()
        cluster_i = gs.vstack([point for point in clusters[i]])
        sphere.add_points(cluster_i)
        sphere.draw(ax=ax, c=color[i, :])


if __name__ == "__main__":
    main()
