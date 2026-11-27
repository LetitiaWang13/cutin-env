"""Optimizer-only tests: real GP/EI numerics, no simulation or safety claims."""

import unittest
import warnings
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

try:
    from cutin.generation.optim.optimizers.bo.optimizer import (
        DEFAULT_BOUNDS,
        PARAMETER_NAMES,
        BOOptimizer,
    )
except ModuleNotFoundError as error:
    if error.name in {"scipy", "sklearn"}:
        raise unittest.SkipTest(
            "install the BO dependencies to run these tests"
        ) from error
    raise


def synthetic_score(point):
    """A deterministic numerical objective, not a driving metric."""
    bounds = np.asarray(DEFAULT_BOUNDS)
    unit = (point - bounds[:, 0]) / (bounds[:, 1] - bounds[:, 0])
    return float(np.sum((unit - [0.25, 0.4, 0.6, 0.7]) ** 2) - 0.1)


class BOOptimizerTests(unittest.TestCase):
    def setUp(self):
        # Kernel-bound warnings are expected on deliberately tiny toy datasets.
        self.warning_context = warnings.catch_warnings()
        self.warning_context.__enter__()
        from sklearn.exceptions import ConvergenceWarning

        warnings.simplefilter("ignore", ConvergenceWarning)

    def tearDown(self):
        self.warning_context.__exit__(None, None, None)

    def make_optimizer(self, **kwargs):
        settings = {
            "n_initial": 3,
            "seed": 17,
            "acquisition_samples": 24,
            "acquisition_restarts": 1,
        }
        settings.update(kwargs)
        return BOOptimizer(**settings)

    def test_default_order_and_bounds(self):
        self.assertEqual(
            PARAMETER_NAMES,
            (
                "lane_change_duration_s",
                "target_gap_m",
                "initial_speed_mps",
                "initial_relative_distance_m",
            ),
        )
        optimizer = BOOptimizer()
        np.testing.assert_equal(optimizer.bounds, DEFAULT_BOUNDS)
        self.assertEqual(optimizer.X.shape, (0, 4))
        self.assertEqual(optimizer.Y.shape, (0, 1))

    def test_30_initial_observations_then_real_gp_ei_proposal(self):
        optimizer = self.make_optimizer(n_initial=30)
        for _ in range(30):
            point = optimizer.suggest()
            self.assertIsNone(optimizer._gp)
            optimizer.update(point, synthetic_score(point))
        point = optimizer.suggest()
        self.assertIsNotNone(optimizer._gp)
        self.assertEqual(optimizer._gp.X_train_.shape, (30, 4))
        self.assertTrue(np.all(optimizer._gp.X_train_ >= 0))
        self.assertTrue(np.all(optimizer._gp.X_train_ <= 1))
        optimizer.update(point, synthetic_score(point))
        self.assertEqual(optimizer.X.shape, (31, 4))
        self.assertEqual(optimizer.Y.shape, (31, 1))

    def test_multiple_real_bo_steps_use_observed_scores(self):
        optimizer = self.make_optimizer()
        for _ in range(6):
            point = optimizer.suggest()
            self.assertTrue(np.all(point >= optimizer.bounds[:, 0]))
            self.assertTrue(np.all(point <= optimizer.bounds[:, 1]))
            score = synthetic_score(point)
            optimizer.update(point, score)
            self.assertEqual(optimizer.Y[-1, 0], score)
        np.testing.assert_allclose(
            optimizer.Y[:, 0], [synthetic_score(x) for x in optimizer.X]
        )
        self.assertEqual(len(np.unique(optimizer.X, axis=0)), 6)

    def test_gp_predictions_change_after_feedback(self):
        optimizer = self.make_optimizer(n_initial=2)
        for _ in range(2):
            point = optimizer.suggest()
            optimizer.update(point, synthetic_score(point))
        point = optimizer.suggest()
        self.assertEqual(len(optimizer._gp.X_train_), 2)
        optimizer.update(point, -1.2)
        optimizer.suggest()
        self.assertEqual(len(optimizer._gp.X_train_), 3)
        self.assertFalse(optimizer._model_dirty)

    def test_expected_improvement_formula_and_zero_sigma_limit(self):
        optimizer = self.make_optimizer()
        optimizer._Y = np.array([[0.0]])
        optimizer._gp = SimpleNamespace(
            predict=lambda X, return_std: (
                np.array([0.0, -1.0, 1.0]),
                np.array([1.0, 0.0, 0.0]),
            )
        )
        ei = optimizer._expected_improvement(np.zeros((3, 4)))
        np.testing.assert_allclose(ei, [1 / np.sqrt(2 * np.pi), 1.0, 0.0])

    def test_seed_reproduces_random_and_bo_sequence(self):
        first = self.make_optimizer(n_initial=2)
        second = self.make_optimizer(n_initial=2)
        for _ in range(4):
            x1, x2 = first.suggest(), second.suggest()
            np.testing.assert_allclose(x1, x2, rtol=0, atol=1e-10)
            first.update(x1, synthetic_score(x1))
            second.update(x2, synthetic_score(x2))

    def test_one_outstanding_suggestion(self):
        optimizer = self.make_optimizer()
        point = optimizer.suggest()
        with self.assertRaises(RuntimeError):
            optimizer.suggest()
        optimizer.update(point, 0.3)
        optimizer.suggest()

    def test_update_without_suggestion_rejected(self):
        with self.assertRaises(RuntimeError):
            self.make_optimizer().update([2, 2, 15, 0], 0.0)

    def test_mismatched_parameters_rejected_without_consuming_pending(self):
        optimizer = self.make_optimizer()
        point = optimizer.suggest()
        changed = point.copy()
        changed[0] += 0.01
        with self.assertRaises(ValueError):
            optimizer.update(changed, 0.0)
        self.assertEqual(len(optimizer.X), 0)
        optimizer.update(point, 0.0)

    def test_invalid_scores_leave_pending_observation_available(self):
        optimizer = self.make_optimizer()
        point = optimizer.suggest()
        for score in (None, np.nan, np.inf, -np.inf, True, [0.1], {"rho": 0.1}, "0.1"):
            with self.subTest(score=score), self.assertRaises(ValueError):
                optimizer.update(point, score)
        self.assertEqual(len(optimizer.Y), 0)
        optimizer.update(point, np.float64(-0.2))
        self.assertEqual(optimizer.Y[0, 0], -0.2)

    def test_invalid_parameter_shapes_rejected(self):
        optimizer = self.make_optimizer()
        point = optimizer.suggest()
        for value in (point[None, :], [1, 2], [np.nan] * 4):
            with self.subTest(value=value), self.assertRaises(ValueError):
                optimizer.update(value, 0.0)

    def test_invalid_constructor_arguments(self):
        cases = (
            {"bounds": [[0, 0]] * 4},
            {"bounds": [[0, np.inf]] * 4},
            {"bounds": [[0, 1]] * 3},
            {"n_initial": 0},
            {"n_initial": True},
            {"acquisition_samples": 0},
            {"acquisition_restarts": 0},
            {"seed": -1},
            {"seed": True},
            {"noise_variance": 0},
            {"noise_variance": np.nan},
            {"noise_variance": True},
        )
        for settings in cases:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                BOOptimizer(**settings)

    def test_returned_arrays_cannot_mutate_internal_state(self):
        optimizer = self.make_optimizer()
        point = optimizer.suggest()
        saved = point.copy()
        point[:] = -100
        optimizer.update(saved, 0.2)
        optimizer.X[:] = -100
        optimizer.Y[:] = -100
        optimizer.bounds[:] = -100
        np.testing.assert_array_equal(optimizer.X[0], saved)
        self.assertEqual(optimizer.Y[0, 0], 0.2)
        np.testing.assert_array_equal(optimizer.bounds, DEFAULT_BOUNDS)

    def test_flat_objective_still_proposes_a_distinct_point(self):
        optimizer = self.make_optimizer(n_initial=2)
        for _ in range(3):
            point = optimizer.suggest()
            optimizer.update(point, 0.0)
        self.assertEqual(len(np.unique(optimizer.X, axis=0)), 3)

    def test_failed_local_refinement_keeps_ei_ranked_pool_candidate(self):
        optimizer = self.make_optimizer(n_initial=2)
        for _ in range(2):
            point = optimizer.suggest()
            optimizer.update(point, synthetic_score(point))
        failure = SimpleNamespace(success=False, x=np.full(4, np.nan))
        target = "cutin.generation.optim.optimizers.bo.optimizer.minimize"
        with patch(target, return_value=failure):
            point = optimizer.suggest()
        optimizer.update(point, synthetic_score(point))
        self.assertEqual(len(optimizer.Y), 3)

    def test_nonfinite_gp_predictions_do_not_issue_a_candidate(self):
        optimizer = self.make_optimizer()
        optimizer._Y = np.array([[0.0]])
        optimizer._gp = SimpleNamespace(
            predict=lambda X, return_std: (np.array([np.nan]), np.array([1.0]))
        )
        with self.assertRaises(ArithmeticError):
            optimizer._expected_improvement(np.zeros((1, 4)))

    def test_custom_physical_bounds_are_used(self):
        bounds = np.array([[3, 4], [4, 5], [20, 21], [30, 31]])
        optimizer = self.make_optimizer(bounds=bounds)
        point = optimizer.suggest()
        self.assertTrue(np.all(point >= bounds[:, 0]))
        self.assertTrue(np.all(point <= bounds[:, 1]))


if __name__ == "__main__":
    unittest.main()
