"""
Unit test for optimal quantization
"""

import unittest

import numpy as np
import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere
from geomstats.hypersphere import HypersphereMetric

TOLERANCE = 5e-3


class TestOptimalQuantization(unittest.TestCase):
    _multiprocess_can_split_ = True

    def setUp(self):
        np.random.seed(1234)
        self.space = 'S2'
        self.metric = HypersphereMetric(dimension=2)
        self.n_points = 1000
        self.points = Hypersphere(dimension=2).random_von_mises_fisher(
                kappa=10,
                n_samples=self.n_points)

    def test_sample_from(self):
        """
        Check that the sample is one of points.
        """
        sample = oq.sample_from(self.points)
        result = False
        for i in range(self.n_points):
            if (self.points[i, :] == sample).all():
                result = True
        self.assertTrue(result)

    def test_closest_neighbor(self):
        """
        Check that the closest neighbor is one of neighbors.
        """
        point = self.points[0, :]
        neighbors = self.points[1:, :]
        index = oq.closest_neighbor(point, neighbors, self.space)
        closest_neighbor = self.points[index, :]
        result = False
        for i in range(self.n_points):
            if (self.points[i, :] == closest_neighbor).all():
                result = True
        self.assertTrue(result)

    def test_karcher_flow_and_optimal_quantization(self):
        """
        Check 1) that the sum of the log maps from the karcher mean
        to the points is close to the zero vector, and
        2) that optimal quantization yields the same result as
        the karcher flow algorithm when we look for one center.
        """
        karcher_mean, n_steps = oq.karcher_flow(self.points, self.space)
        tangent_vectors = np.zeros(self.points.shape)
        for i in range(self.n_points):
            tangent_vectors[i, :] = self.metric.log(
                    self.points[i, :], karcher_mean)
        sum_tan_vec = np.sum(tangent_vectors, axis=0)
        result = self.metric.squared_norm(sum_tan_vec)
        expected = 0.0

        self.assertTrue(np.allclose(result, expected))

        n_centers = 1
        centers, weights, clustering, n_steps = oq.optimal_quantization(
            self.points, n_centers, self.space
            )
        error = self.metric.dist(karcher_mean, centers)
        diameter = oq.diameter_of_data(self.points, self.space)
        result = error / diameter
        expected = 0.0

        self.assertTrue(np.allclose(result, expected, atol=TOLERANCE))


if __name__ == '__main__':
        unittest.main()
