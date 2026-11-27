"""Sequential GP/EI proposals; simulation and evaluation belong to the generator.

Install the BO dependencies in the team environment with:
    uv add --optional bo 'scikit-learn>=1.8,<2' 'scipy>=1.13,<2'
    uv sync --extra bo

The implementation uses a standard Gaussian-process surrogate and Expected
Improvement for minimization. It does not define safety metrics or robustness.
"""

from __future__ import annotations

from numbers import Integral, Real

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.optimize import minimize
from scipy.special import ndtr
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel

from cutin.generation.optim.optimizers.interface import Optimizer

# These draft bounds are experimental, not safety-calibrated or jointly feasible.
PARAMETER_NAMES = (
    "lane_change_duration_s",
    "target_gap_m",
    "initial_speed_mps",
    "initial_relative_distance_m",
)
DEFAULT_BOUNDS = ((2.0, 5.0), (2.0, 20.0), (15.0, 40.0), (0.0, 40.0))


class BOOptimizer(Optimizer):
    """Propose one physical parameter vector, then accept one observed scalar.

    Order: planned lateral-reference duration [s], merge-start bumper gap [m],
    initial cut-in speed [m/s], initial center offset x_cutin - x_ego [m].
    The generator must resolve the offset into world x in ScenarioConfig.

    The first n_initial observations use uniform random proposals. Later calls
    fit a GP and maximize standard EI using a random candidate pool followed by
    bounded local refinement. Inputs are scaled to [0, 1]^4 internally; callers
    always send and receive physical values. Outputs are standardized by the GP.

    update(parameters, metrics) expects metrics to be ONE finite observed score
    to MINIMIZE, e.g. agreed safety robustness. It does not accept a metrics dict,
    a TTC/LCT pair, or an unavailable score. A negative score is not interpreted
    here: safety specifications and result selection belong outside this class.

    Only one outstanding suggestion is allowed. There is no simulation loop,
    automatic retry, feasibility oracle, persistence, or stopping budget here.
    The GP is refitted lazily on the next BO suggestion after new feedback.
    """

    def __init__(
        self,
        bounds: ArrayLike = DEFAULT_BOUNDS,
        *,
        n_initial: int = 30,
        seed: int | None = 17,
        noise_variance: float = 1e-6,
        acquisition_samples: int = 256,
        acquisition_restarts: int = 5,
    ) -> None:
        self._bounds = np.asarray(bounds, dtype=float).copy()
        if self._bounds.shape != (4, 2) or not np.isfinite(self._bounds).all():
            raise ValueError("bounds must be a finite (4, 2) array")
        self._lower = self._bounds[:, 0]
        self._span = self._bounds[:, 1] - self._lower
        if not np.isfinite(self._span).all() or np.any(self._span <= 0):
            raise ValueError("every search interval must have positive finite width")

        for name, value in (
            ("n_initial", n_initial),
            ("acquisition_samples", acquisition_samples),
            ("acquisition_restarts", acquisition_restarts),
        ):
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if seed is not None and (
            isinstance(seed, bool) or not isinstance(seed, Integral) or seed < 0
        ):
            raise ValueError("seed must be a non-negative integer or None")
        if (
            isinstance(noise_variance, bool)
            or not isinstance(noise_variance, Real)
            or not np.isfinite(noise_variance)
            or noise_variance <= 0
        ):
            raise ValueError("noise_variance must be a positive finite scalar")

        self._n_initial = int(n_initial)
        self._acquisition_samples = int(acquisition_samples)
        self._acquisition_restarts = int(acquisition_restarts)
        # Numerical regularization in standardized-Y units, not a safety threshold.
        self._noise_variance = float(noise_variance)
        self._rng = np.random.default_rng(seed)
        self._X = np.empty((0, 4), dtype=float)
        self._Y = np.empty((0, 1), dtype=float)
        self._pending: NDArray[np.float64] | None = None
        self._gp: GaussianProcessRegressor | None = None
        self._model_dirty = True

    @property
    def bounds(self) -> NDArray[np.float64]:
        """Return a copy of the physical search bounds."""
        return self._bounds.copy()

    @property
    def X(self) -> NDArray[np.float64]:
        """Observed physical parameters, shape (N, 4); returned as a copy."""
        return self._X.copy()

    @property
    def Y(self) -> NDArray[np.float64]:
        """Observed scalar scores, shape (N, 1); never GP or EI predictions."""
        return self._Y.copy()

    def suggest(self) -> NDArray[np.float64]:
        """Return one (4,) physical vector; call update before suggesting again."""
        if self._pending is not None:
            raise RuntimeError("call update for the outstanding suggestion first")

        if len(self._X) < self._n_initial:
            for _ in range(1000):
                unit_point = self._rng.random(4)
                if self._is_novel(unit_point[None, :])[0]:
                    break
            else:
                raise RuntimeError("could not draw a distinct initial candidate")
        else:
            if self._model_dirty:
                self._fit_gp()
            unit_point = self._maximize_ei()

        point = self._lower + self._span * unit_point
        self._pending = point.copy()
        return point

    def update(self, parameters: ArrayLike, metrics: float) -> None:
        """Append the pending parameters and their finite observed score.

        Invalid feedback leaves observations and the pending suggestion intact.
        Keep the proposed vector unchanged when constructing the scenario. Extract
        the scalar robustness from the evaluator's result outside the optimizer.
        """
        if self._pending is None:
            raise RuntimeError("update requires an outstanding suggestion")
        point = np.asarray(parameters, dtype=float)
        if point.shape != (4,) or not np.isfinite(point).all():
            raise ValueError("parameters must be a finite vector of shape (4,)")
        if not np.array_equal(point, self._pending):
            raise ValueError("feedback parameters do not match the suggestion")
        if (
            isinstance(metrics, (bool, np.bool_))
            or not isinstance(metrics, Real)
            or not np.isfinite(metrics)
        ):
            raise ValueError("metrics must be one finite observed scalar to minimize")

        self._X = np.vstack((self._X, point))
        self._Y = np.vstack((self._Y, [[float(metrics)]]))
        self._model_dirty = True
        self._pending = None

    def _fit_gp(self) -> None:
        """Fit an ARD RBF surrogate; GP fitting does not execute a scenario."""
        kernel = (
            self._gp.kernel_
            if self._gp is not None
            else ConstantKernel(1.0, (1e-3, 1e3)) * RBF(np.ones(4), (1e-2, 1e2))
        )
        gp = GaussianProcessRegressor(
            kernel=kernel,
            alpha=self._noise_variance,
            normalize_y=True,
            n_restarts_optimizer=0,
        )
        gp.fit((self._X - self._lower) / self._span, self._Y[:, 0])
        self._gp = gp
        self._model_dirty = False

    def _expected_improvement(self, unit_points: NDArray) -> NDArray[np.float64]:
        """Standard EI = E[max(y_best - f(x), 0)], in observed-score units."""
        if self._gp is None:
            raise RuntimeError("the GP must be fitted before computing EI")
        mean, std = self._gp.predict(unit_points, return_std=True)
        if not np.isfinite(mean).all() or not np.isfinite(std).all():
            raise ArithmeticError("non-finite GP prediction; no candidate was issued")
        if np.any(std < 0):
            raise ArithmeticError("the GP returned a negative standard deviation")

        improvement = float(self._Y.min()) - mean
        ei = np.maximum(improvement, 0.0)  # Deterministic limit when sigma is zero.
        uncertain = std > 1e-12
        z = improvement[uncertain] / std[uncertain]
        density = np.exp(-0.5 * z**2) / np.sqrt(2.0 * np.pi)
        ei[uncertain] = improvement[uncertain] * ndtr(z) + std[uncertain] * density
        if not np.isfinite(ei).all():
            raise ArithmeticError("non-finite EI; no candidate was issued")
        return np.maximum(ei, 0.0)

    def _is_novel(self, unit_points: NDArray) -> NDArray[np.bool_]:
        if len(self._X) == 0:
            return np.ones(len(unit_points), dtype=bool)
        # Round-trip through physical units before checking for duplicate vectors.
        resolved = self._lower + self._span * unit_points
        unit_points = (resolved - self._lower) / self._span
        observed = (self._X - self._lower) / self._span
        distances = np.linalg.norm(
            unit_points[:, None, :] - observed[None, :, :], axis=2
        )
        return np.all(distances > 1e-8, axis=1)

    def _maximize_ei(self) -> NDArray[np.float64]:
        """Maximize EI approximately; all expensive evaluations happen elsewhere."""
        pool = self._rng.random((self._acquisition_samples, 4))
        pool = pool[self._is_novel(pool)]
        if len(pool) == 0:
            raise RuntimeError("no distinct acquisition candidates; adjust the space")
        values = self._expected_improvement(pool)
        ranking = np.argsort(values)[::-1]
        best_point = pool[ranking[0]].copy()
        best_ei = float(values[ranking[0]])

        def negative_ei(unit_point):
            return -float(self._expected_improvement(unit_point[None, :])[0])

        for index in ranking[: self._acquisition_restarts]:
            result = minimize(
                negative_ei,
                pool[index],
                method="L-BFGS-B",
                bounds=[(0.0, 1.0)] * 4,
                options={"maxiter": 100},
            )
            point = np.asarray(result.x, dtype=float)
            if (
                not result.success
                or not np.isfinite(point).all()
                or np.any(point < 0)
                or np.any(point > 1)
                or not self._is_novel(point[None, :])[0]
            ):
                continue  # The EI-ranked pool remains a usable fallback.
            value = float(self._expected_improvement(point[None, :])[0])
            if value > best_ei:
                best_point, best_ei = point.copy(), value
        return best_point
