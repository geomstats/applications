#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit test for optimal quantization
"""

import unittest

import geomstats.backend as gs
import quantization.optimal_quantization as oq

from geomstats.hypersphere import Hypersphere
from geomstats.hypersphere import HypersphereMetric

TOLERANCE = 5e-3


class TestOptimalQuantization(unittest.TestCase):
    _multiprocess_can_split_ = True

    def setUp(self):
        gs.random.seed(1234)
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
            if gs.allclose(self.points[i, :], sample):
                result = True
                break
        self.assertTrue(result)

    def test_optimal_quantization(self):
        """
        Check that optimal quantization yields the same result as
        the karcher flow algorithm when we look for one center.
        """
        n_centers = 1
        mean = self.metric.mean(self.points)
        centers, weights, clusters, n_iterations = oq.optimal_quantization(
            points=self.points, metric=self.metric, n_centers=n_centers
            )
        error = self.metric.dist(mean, centers)
        diameter = self.metric.diameter(self.points)
        result = error / diameter
        expected = 0.0

        self.assertTrue(gs.allclose(result, expected, atol=TOLERANCE))


if __name__ == '__main__':
        unittest.main()
