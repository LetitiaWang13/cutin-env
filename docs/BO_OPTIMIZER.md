# Bayesian optimizer component

## Scope

`BOOptimizer` extends the original `Optimizer` interface without adding a
simulation loop. It proposes one physical parameter vector through `suggest()`
and receives its observed scalar score through `update(parameters, metrics)`.
The scalar is minimized. The existing generator remains the owner of simulation,
evaluation, stopping, record collection, and result selection.

This component does not implement TTC, LCT, robustness, aggressiveness mapping,
joint scenario validation, or `OptimGenerator.run()`.

## Dependencies and setup

Run these commands from the repository root in the team's normal environment:

```sh
uv add --optional bo 'scikit-learn>=1.8,<2' 'scipy>=1.13,<2'
uv sync --extra bo
uv run --extra bo python -m unittest discover -s tests/generation -p test_bo.py -v
uv run --extra bo pytest -q
uv run --extra bo ruff check .
uv run --extra bo ruff format --check .
```

The first command adds the optional extra to `pyproject.toml` and resolves
`uv.lock`; retain both resulting files when integrating the component. The
provided update intentionally leaves the uploaded manifest and lockfile unchanged:
the authoring environment returned HTTP 403 for package-index access, including
attempts to install the original project/build dependencies. No restricted
package source or alternate credentials were used.

The tested numerical backend was scikit-learn 1.8.0, SciPy 1.17.0, NumPy 2.3.5,
and Python 3.12.14, already available in that environment. These are not a new
project lockfile or a cross-platform compatibility guarantee. The earlier
standalone tutorial used GPy/Emukit; this repository component uses scikit-learn
and SciPy to make the actual numerical path testable without downloading packages.
Both approaches implement the same GP / standard EI / acquisition-search roles.

## Candidate and feedback contract

`suggest()` returns a NumPy array with shape `(4,)`, in this exact order:

| Index | Name | Unit | Draft interval | Meaning |
|---|---|---|---|---|
| 0 | `lane_change_duration_s` | s | [2, 5] | Planned lateral-reference duration, not actual LCT |
| 1 | `target_gap_m` | m | [2, 20] | Merge-start bumper-gap trigger, not final merge gap |
| 2 | `initial_speed_mps` | m/s | [15, 40] | Cut-in initial speed and pre-FOLLOW speed target |
| 3 | `initial_relative_distance_m` | m | [0, 40] | Initial center offset: x_cutin - x_ego |

These are the user's provisional intervals, not calibrated safety thresholds.
They constrain individual dimensions, not joint feasibility or merge reachability.
In particular, the controller waits for the gap trigger after warmup; it does not
adjust its speed specifically to reach an arbitrary target gap.

Pass the unmodified proposed vector and ONE finite observed scalar to `update`.
Extract that scalar from the evaluator's result outside this class. Dictionaries,
TTC/LCT pairs, booleans, `None`, NaN, and infinity are rejected. No arbitrary
penalty or replacement score is invented. Invalid feedback leaves the pending
point and all prior observations intact so the integration error can be corrected.

Only one suggestion can be outstanding. There is no batch/asynchronous proposal,
observation preload, discard/retry API, or resumable checkpoint in this version.
An unscorable run requires an explicit generator/evaluator policy before the
experiment can continue; do not classify it as safe or fabricate GP feedback.

## What each code block does

1. **Bounds and validation:** enforce four finite, positive-width input intervals.
2. **Observation state:** preserve physical `X: (N, 4)` and observed `Y: (N, 1)`.
   Public array properties return copies rather than mutable internal storage.
3. **Initial design:** uniform random proposals using an instance-local seeded RNG.
   The default initialization size is 30 valid observed parameter-score pairs.
4. **GP:** normalize inputs to the unit cube; fit an ARD RBF Gaussian process;
   standardize Y. `noise_variance` is numerical/model regularization in normalized
   Y units, not a safety threshold or a calibrated noise estimate.
5. **Acquisition:** evaluate standard Expected Improvement for minimization:
   `E[max(y_best - f(x), 0)]`. This is not a new driving-specific risk score.
6. **Acquisition optimization:** maximize EI approximately by ranking random
   candidates and locally minimizing negative EI with bounded L-BFGS-B. If local
   refinement fails or repeats an observed point, keep a novel EI-ranked pool
   candidate. No simulator call is made inside this search.
7. **Feedback:** append only the actual scalar returned by evaluation; refit the
   GP lazily at the next BO `suggest()`. GP predictions and EI never enter Y.

## Minimal runnable numerical example

After installing the dependencies, this code runs the actual GP/EI optimizer on
a fake numerical objective. It is not a driving simulation or safety experiment.

```python
import numpy as np

from cutin.generation.optim.optimizers.bo.optimizer import BOOptimizer

optimizer = BOOptimizer(n_initial=30, seed=17)
bounds = optimizer.bounds

for _ in range(30 + 70):
    parameters = optimizer.suggest()
    unit = (parameters - bounds[:, 0]) / (bounds[:, 1] - bounds[:, 0])
    score = float(np.sum((unit - [0.25, 0.4, 0.6, 0.7]) ** 2) - 0.1)
    optimizer.update(parameters, score)

best = int(np.argmin(optimizer.Y[:, 0]))
print(optimizer.X.shape, optimizer.Y.shape)  # (100, 4), (100, 1)
print(optimizer.X[best], optimizer.Y[best, 0])
```

The optimizer does not contain a hardcoded 70-iteration budget. The caller owns
that budget: 30 initial observations plus 70 BO observations means 100 successful
objective evaluations, not 100 BO iterations after initialization. Rejected or
unscorable attempts need separate accounting at the generator level.

## Connecting the existing simulator

The generator should use a small candidate-to-scenario conversion function.
For example, the following mapping preserves the base scenario rather than
mutating shared fixed conditions:

```python
from dataclasses import replace


def candidate_to_scenario(base, parameters):
    duration, gap, speed, offset = map(float, parameters)
    return replace(
        base,
        cutin=replace(base.cutin, x_m=base.ego.x_m + offset, speed_mps=speed),
        cutin_controller=replace(
            base.cutin_controller,
            target_gap_m=gap,
            lane_change_duration_s=duration,
        ),
    )
```

This demonstrates mapping only, not geometry or warmup-reachability validation.
The generator must then execute the existing adapter and evaluator, extract the
agreed robustness scalar, and call `optimizer.update(parameters, rho)`.
Retain the scenario, record, evaluation status, and score together outside the
optimizer for filtering and replay. The current evaluator/generator placeholders
must still be implemented for a real end-to-end experiment.

If the objective changes from falsification to matching an aggressiveness target,
define an appropriate scalar matching loss outside this optimizer. Minimizing
safety robustness is not automatically aggressiveness matching.

## Limits and verification

The acquisition optimizer is approximate, the GP is a model, and the bounds are
experimental. No globally optimal result, coverage guarantee, or safety
certification is implied. Use equal-budget, multi-seed random-search comparisons
before claiming an advantage for BO. Bitwise cross-platform reproducibility is
not promised.

The component tests cover the real numerical GP/EI path as well as input contracts,
seeding, flat outputs, non-finite feedback, pending-point integrity, duplicate
avoidance, and failed local-refinement fallback. A few targeted failure/formula
tests use explicit test doubles; the real numerical tests do not.

Verification in the authoring environment:

- All 17 component tests passed.
- A separate run completed 30 random observations followed by 70 actual GP/EI
  proposals, with 100 distinct parameter vectors and finite observed scores.
- The fake quadratic objective's best observed score changed from approximately
  -0.06118 after initialization to -0.09999 after all 100 observations. This is a
  numerical smoke result, not evidence of driving safety or an equal-budget
  optimization comparison.

The original repository's full pytest/build/Ruff checks were not run: project
dependency installation was blocked by HTTP 403 and Ruff was absent from the
offline cache. The delivered source was syntax-checked and tested independently
against the already installed numerical stack through `uv run --offline
--no-project`. No real simulator or robustness evaluation was executed.
