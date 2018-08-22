"""
Plot the result of optimal quantization of the von Mises Fisher distribution
on the sphere
"""

import matplotlib.pyplot as plt
import numpy as np

import geomstats.visualization as visualization
import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere

SPHERE2 = Hypersphere(dimension=2)


def main():
    # generate points from the von Mises Fisher distribution
    n_points = 1000
    points = SPHERE2.random_von_mises_fisher(kappa=10, n_samples=n_points)

    # find the 5 centers that best approximate the points
    n_centers = 5
    centers, weights, clustering, n_steps = oq.optimal_quantization(
                points, n_centers, space='S2'
                )
    plt.figure(0)
    ax = plt.subplot(111, projection="3d", aspect="equal")
    visualization.plot(centers, ax, space='S2', c='r')
    plt.show()

    # the Voronoi diagram associated to the centers yields a clustering
    clusters = {}
    for i in range(n_centers):
        clusters[i] = points[clustering == i]

    sphere = visualization.Sphere()

    plt.figure(1)
    ax = plt.subplot(111, projection="3d", aspect="equal")
    color = np.random.rand(n_centers, 3)
    ax.plot_wireframe(sphere.sphere_x,
                      sphere.sphere_y,
                      sphere.sphere_z,
                      color="black", alpha=0.2)
    for i in range(n_centers):
        points_x = np.vstack([point[0] for point in clusters[i]])
        points_y = np.vstack([point[1] for point in clusters[i]])
        points_z = np.vstack([point[2] for point in clusters[i]])
        ax.scatter(points_x, points_y, points_z,  c=color[i, :])


if __name__ == "__main__":
    main()
