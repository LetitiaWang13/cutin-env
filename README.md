# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for delivery order.

## Current scope

The foundation has scenario parsing/validation and small immutable records.
There is no simulation, CLI, or visualization yet. No runtime dependencies;
all numerical defaults are illustrative, not safety-calibrated.

```sh
uv sync --locked
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv build
```

## Try the configuration locally

From this repository directory:

```sh
uv run python - <<'PY'
import tomllib
from dataclasses import replace
from pathlib import Path

from cutin.core import ScenarioConfig, validate_scenario

scenario = ScenarioConfig.from_dict(
    tomllib.loads(Path("examples/baseline.toml").read_text())
)
print("Initial conditions:", scenario.initial)
print("Cut-in:", scenario.cut_in)

changed = replace(scenario, cut_in=replace(scenario.cut_in, start_time_s=3.0))
validate_scenario(changed)
print("Changed cut-in:", changed.cut_in)

try:
    ScenarioConfig.from_dict({"cut_in": {"lane_change_duration_s": 0}})
except ValueError as error:
    print("Validation error:", error)
PY
```

## Reading the code

- `core/scenario.py`: scenario records plus a single `validate_scenario()` boundary.
  `from_dict()` parses TOML/JSON values and validates automatically. Python-created
  or modified records must be validated explicitly before execution. Constructors
  do not perform hidden checks. Unknown fields and invalid external values fail.
- `core/state.py`: vehicle roles/states and `Snapshot(tick, vehicles)`.
  Integer tick is the authoritative simulation clock; seconds = tick / physics_hz.
- `core/action.py`: acceleration/steering requests and role-keyed applied actions.
- `core/log.py`: scenario, sampled state/actions, termination, optional diagnostic.
  No event framework or per-constructor lifecycle validation.

TOML is for hand-edited scenarios; `to_dict()` expands defaults into JSON-compatible
values for future API/artifact output. Core performs no file I/O. Three-vehicle
restrictions live only in scenario validation, not in generic snapshots/logs.
The runner will own internal record consistency and validate Python-created
scenario inputs before execution; backend settings and log serialization follow
in the integration probe.

Coordinates use SI units: x follows the road, lane centres are
`lane * lane_width_m`, and heading is radians from positive x. Positions are
vehicle centres. `start_time_s` begins the lateral reference at the next control
tick; PREPARE is deferred. Requested lane-change duration is not guaranteed
physical completion time.
