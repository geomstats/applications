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

N_CENTERS = 10
TOLERANCE = 1e-5
N_REPETITIONS = 20
N_MAX_ITERATIONS = 50000

# TODO(alice): Consider adding optimal_quantization
# as a method of the class RiemannianMetric.


def sample_from(points, n_samples=1):
    """
    Sample from the empirical distribution associated to points.
    """
    n_points = points.shape[0]
    dimension = points.shape[-1]

    index = gs.random.randint(low=0, high=n_points, size=(n_samples,))
    sample = points[gs.ix_(index, gs.arange(dimension))]

    return sample


def optimal_quantization(points, metric, n_centers=N_CENTERS,
                         n_repetitions=N_REPETITIONS,
                         tolerance=TOLERANCE,
                         n_max_iterations=N_MAX_ITERATIONS):
    """
    Compute the optimal approximation of points by a smaller number
    of weighted centers using the Competitive Learning Riemannian
    Quantization algorithm. The centers are updated using decreasing
    step sizes, each of which stays constant for n_repetitions iterations
    to allow a better exploration of the data points.
    See https://arxiv.org/abs/1806.07605.
    Return :
        - n_centers centers
        - n_centers weights between 0 and 1
        - a dictionary containing the clusters, where each key is the
          cluster index, and its value is the lists of points belonging
          to the cluster
        - the number of steps needed to converge.
    """
    centers = sample_from(points, n_centers)

    gap = 1.0
    iteration = 0

    while iteration < n_max_iterations:
        iteration += 1
        step_size = gs.floor(iteration / n_repetitions) + 1

        point = sample_from(points)

        index = metric.closest_neighbor_index(point, centers)
        center_to_update = centers[index, :]

        tangent_vec_update = metric.log(
                point=point, base_point=center_to_update
                ) / (step_size+1)
        new_center = metric.exp(
                tangent_vec=tangent_vec_update, base_point=center_to_update
                )
        gap = metric.dist(center_to_update, new_center)
        gap = (gap != 0) * gap + (gap == 0)

        centers[index, :] = new_center

        if gs.isclose(gap, 0, atol=tolerance):
                break

    if iteration == n_max_iterations-1:
        print('Maximum number of iterations {} reached. The'
              'quantization may be inaccurate'.format(n_max_iterations))

    n_points = points.shape[0]
    clusters = dict()
    weights = gs.zeros((n_centers,))
    index_list = list()

    for point in points:
        index = metric.closest_neighbor_index(point, centers)
        if index not in index_list:
            clusters[index] = list()
            index_list.append(index)
        clusters[index].append(point)
        weights[index] += 1

    weights = weights / n_points

    return centers, weights, clusters, iteration
