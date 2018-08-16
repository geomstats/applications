"""
Optimal quantization
"""

import numpy as np
from geomstats.hypersphere import HypersphereMetric


S2metric = HypersphereMetric(dimension=2)

TOLERANCE = 1e-5
IMPLEMENTED = ['S2']


def sample_from(points, size=1):
    """
    Sample from the empirical distribution associated to points
    """
    n_points = points.shape[0]
    dimension = points.shape[-1]

    ind = np.random.randint(low=0, high=n_points, size=size)
    sample = points[np.ix_(ind, np.arange(dimension))]

    return sample


def closest_neighbor(point, neighbors, space=None):
    """
    Find closest neighbor of point among neighbors
    """
    if space == 'S2':
        dist = S2metric.dist(point, neighbors)
        ind_closest_neighbor = dist.argmin()

    return ind_closest_neighbor


def optimal_quantization(points, n_centers=10, space=None, n_repetition=20,
                         tolerance=TOLERANCE):
    """
    Compute the optimal approximation of points by a smaller number
    n of weighted centers.
    """
    if space not in IMPLEMENTED:
        raise NotImplementedError(
                'The optimal quantization function is not implemented'
                ' for space {}. The spaces available for quantization'
                ' are: {}.'.format(space, IMPLEMENTED))
    # random initialization of the centers
    centers = sample_from(points, n_centers)

    gap = 1
    step = 0

    while gap > tolerance:
        step += 1
        k = np.floor(step / n_repetition) + 1

        sample = sample_from(points)

        ind = closest_neighbor(sample, centers, space)
        center_to_update = centers[ind, :]

        if space == 'S2':
            tan_vec_update = S2metric.log(sample, center_to_update) / (k+1)
            new_center = S2metric.exp(tan_vec_update, center_to_update)
            gap = S2metric.dist(center_to_update, new_center)

        centers[ind, :] = new_center

    return centers, step
