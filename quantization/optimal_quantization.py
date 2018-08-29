"""
Optimal quantization of the empirical distribution of a dataset.

The algorithm finds an optimal aproximation of the dataset by a
smaller number n of points, and yields a K-means type clustering.
The n points of the approximation, weighted by the masses of their
Voronoi cells, define a discrete distribution that best approximates,
in the sense of the Wasserstein distance and at fixed support size n,
the empirical distribution of the dataset. The Voronoi diagram associated
to these points yields a K-means type clustering of the dataset.
"""

import geomstats.backend as gs
from geomstats.hypersphere import HypersphereMetric


TOLERANCE = 1e-5
IMPLEMENTED = ('S1', 'S2')


def sample_from(points, size=1):
    """
    Sample from the empirical distribution associated to points.
    """
    n_points = points.shape[0]
    dimension = points.shape[-1]

    index = gs.random.randint(low=0, high=n_points, size=size)
    sample = points[gs.ix_(index, gs.arange(dimension))]

    return sample


def optimal_quantization(points, n_centers=10, space=None, n_repetitions=20,
                         tolerance=TOLERANCE, n_max_iterations=50000):
    """
    Compute the optimal approximation of points by a smaller number
    of weighted centers using the Competitive Learning Riemannian
    Quantization algorithm. See https://arxiv.org/abs/1806.07605.
    Return :
        - n_centers centers
        - n_centers weights between 0 and 1
        - an array of labels defining a clustering
        - the number of steps needed to converge.
    """
    if space not in IMPLEMENTED:
        raise NotImplementedError(
                'The optimal quantization function is not implemented'
                ' for space {}. The spaces available for quantization'
                ' are: {}.'.format(space, IMPLEMENTED))
    elif space == 'S1':
        metric = HypersphereMetric(dimension=1)
    else:
        metric = HypersphereMetric(dimension=2)

    # random initialization of the centers
    centers = sample_from(points, n_centers)

    gap = 1.0
    iteration = 0

    while iteration < n_max_iterations:
        iteration += 1
        step_size = gs.floor(iteration / n_repetitions) + 1

        sample = sample_from(points)

        index = metric.closest_neighbor(sample, centers)
        center_to_update = centers[index, :]

        tangent_vec_update = metric.log(sample, center_to_update) \
            / (step_size+1)
        new_center = metric.exp(tangent_vec_update, center_to_update)
        gap = metric.dist(center_to_update, new_center)
        gap = (gap != 0) * gap + (gap == 0)

        centers[index, :] = new_center

        if gs.isclose(gap, 0, atol=tolerance):
                break

    if iteration is n_max_iterations:
        print('Maximum number of iterations {} reached. The'
              'quantization may be inaccurate'.format(n_max_iterations))

    n_points = points.shape[0]
    clusters = dict()
    weights = gs.zeros((n_centers,))
    index_list = list()

    for i, point in enumerate(points):
        index = metric.closest_neighbor(point, centers)
        if index not in index_list:
            clusters[index] = list()
            index_list.append(index)
        clusters[index].append(i)
        weights[index] += 1

    weights = weights / n_points

    return centers, weights, clusters, iteration
