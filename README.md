# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for delivery order.

## Current scope

The runner executes configured car-following, lane keeping, and cut-in policies
through CutInEnv(HighwayEnv), with optional native visualization. Metrics, a
packaged CLI command, and web visualization are not implemented yet. Numerical defaults are experimental,
not safety-calibrated.
Only highway-env adapter/translation modules import the backend. Core and the
runner's public imports remain backend-independent.
Runtime dependencies are highway-env, Gymnasium, and NumPy (plus FOSS dependencies).
Gym is used only for environment/rendering interfaces, not RL training.

```sh
uv sync --locked
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv build
```

## Run a configured scenario

From this repository directory:

```sh
uv sync --locked
uv run python scripts/run_scenario.py
uv run python scripts/run_scenario.py examples/unsafe.toml
# Optional log export; existing files are never overwritten.
uv run python scripts/run_scenario.py --output runs/baseline.json
```

The baseline runs 10 seconds and completes a lane-0-to-lane-1 merge without
collision in the locked setup. The unsafe fixture deliberately causes an
**ego/cut-in collision**, not a front-only collision. These are regression
examples, not a safety guarantee. Results print final states, termination, and
actual cut-in reference-start/completion ticks. See native visualization below.

`simulate(scenario, SimulatorConfig())` executes policies by default. Supplying
explicit `actions=...` selects fixed-action diagnostic mode instead.

## Native highway-env visualization

On a machine with a desktop display:

```sh
uv run python scripts/run_scenario.py --render
uv run python scripts/run_scenario.py examples/unsafe.toml --render
# The same log export works during visual runs.
uv run python scripts/run_scenario.py --render --output runs/visual-baseline.json
```

This uses highway-env's own EnvViewer, not a custom Pygame GUI or Canvas renderer.
Blue is ego, green is front, orange is cut-in (native collision styling may
change colors). Physics remains fixed-step; human display is paced at the
physics frequency. Slow rendering slows wall-clock playback, never skips ticks.

- Close the window during a run to cancel and retain the partial log.
- After collision/horizon/road exit, the final scene stays until you close it.
- Keyboard driving is disabled; the official camera zoom/pan keys remain native.
- No form controls, pause/single-step, or training interface is added.
- Only one native viewer at a time is supported; upstream closes global Pygame.

Headless runs do not initialize a viewer/display. `render_mode="rgb_array"` on
HighwayEnvAdapter returns native RGB pixels for automated smoke checks. Baseline
and unsafe rendered logs match headless logs exactly; the Gym wrapper also
matches the previous primitive stepping path at 20/30 Hz. Automated SDL-dummy
checks verify drawing, keyboard isolation, close/cancel, and cleanup; confirm
actual desktop appearance/input with the commands above.

## Run the raw-action physics probe

From this repository directory:

```sh
uv sync --locked
uv run python scripts/probe_highway_env.py
# Optional: save resolved settings, initial/terminal states, and applied actions.
uv run python scripts/probe_highway_env.py --output runs/probe.json
# Coast for the whole 10-second horizon, without steering/acceleration.
uv run python scripts/probe_highway_env.py --steps 300 --acceleration 0 --steering 0
```

The default probe advances 30 physics ticks (one simulated second). Ego/front
coast; cut-in receives raw acceleration 1 m/s^2 and steering 0.02 rad. It prints
initial/final states and termination. No display server or window is needed.
The step-limited probe reports `cancelled`, not a completed safety assessment.
Collisions/road exit stop it early. Existing output files are never overwritten.

**This probe bypasses the scenario controller:** it deliberately does not execute
cut-in start time, lane-change duration, or car-following policy. Use
`scripts/run_scenario.py` for those.
JSON export records the scenario, simulator settings/backend version, integer
ticks, actual bounded actions, and per-vehicle collision flags. Metadata is plain
JSON-compatible data, not a backend object or an event framework.

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

- `core/scenario_config.py`: scenario records plus a single `validate_scenario()` boundary.
  `from_dict()` parses TOML/JSON values and validates automatically. Python-created
  or modified records must be validated explicitly before execution. Constructors
  do not perform hidden checks. Unknown fields and invalid external values fail.
- `core/state.py`: vehicle roles/states and `StateSnapshot(tick, vehicles)`.
  Integer tick is the authoritative simulation clock; seconds = tick / physics_hz.
  `vehicles` is a flat tuple, supporting multiple vehicles per role.
  `snapshot.vehicles_for_role(Role.FRONT)` returns all matches as a tuple, or `()`.
  Producers supply tuples; there are no container wrappers or constructor hooks.
- `core/action.py`: acceleration/steering requests and role-keyed applied actions.
- `core/log.py`: scenario, entries, `termination_reason` (TerminationReason enum),
  optional diagnostic, resolved simulator settings, and CutInReport with actual
  reference-start/completion ticks. Null completion means not observed, not safe.
  No event framework or per-constructor lifecycle validation.

TOML is for hand-edited scenarios; `to_dict()` expands defaults into JSON-compatible
values for future API/artifact output. Core performs no file I/O. Three-vehicle
restrictions live only in scenario validation, not in generic snapshots/logs.
The simulation component is now split into:

- `simulation/config.py`: SimulatorConfig (frequencies and generic actuator limits).
- `simulation/runner.py`: `simulate()` validates inputs, supplies dt, bounds generic
  actions, decides termination, assembles the log, and closes the adapter. Every
  controller tick uses one shared snapshot; actions are held across physics ticks.
- `simulation/controllers.py`: pure following/lateral/reference functions and
  one fresh DrivingPolicy per run. No controller registration or state framework.
- `simulation/adapters/highway_env_translation.py`: `build_scene()` translates a
  validated scenario into backend Road/Vehicle objects; `translate_actions()` maps
  action fields and enforces backend speed limits; `collect_snapshot()` copies
  backend state into our records. No Translator classes or config framework.
- `simulation/adapters/highway_env.py`: HighwayEnvAdapter coordinates those
  translations and bridges our records to CutInEnv. It accepts `step(actions, dt_s)`
  and has no SimulatorConfig dependency. It exposes optional native render/close.
- `simulation/adapters/cut_in_env.py`: CutInEnv subclasses HighwayEnv but overrides
  stock reset/step/spaces. It builds only our scene, applies joint physical actions,
  and advances exactly one Road.step per call. Its integer steps are the one
  physics clock; the runner does not maintain another counter. Reward is zero.
  Native reset/step render automatically in human mode; rgb_array is explicit.

The probe is now only argument parsing, calling the same `simulate()` entry
point, and displaying/exporting the returned log, not a second simulation loop.
The runner's small Protocol allows fake-adapter tests without backend objects.
An optional on_step(snapshot) observer receives the initial and each physics
snapshot, returns False to cancel, and keeps display/input outside scheduling.

The baseline policy is bounded IDM-style following plus bicycle-model lateral
tracking (`policy_version=baseline-v1` in controlled logs). Leaders are selected
by projected footprint overlap, including partial intrusion, not lane labels.
Cut-in tracks its target speed during WAIT/MERGE without vetoing an unsafe merge;
after actual completion it follows the same baseline policy as ego/front.
Completion requires the reference to finish, lateral error <= 0.2 m, heading
error <= 0.05 rad, and no cut-in collision. Unstarted/incomplete maneuvers remain
explicit in the report. Steering cannot move a stationary vehicle.

Requested start time quantizes to the next control tick. The actual reference
anchors at that tick; requested duration does not guarantee tracking completion.
Termination and maneuver observations are not safety metrics.

The probe supports highway-env's default 5 m x 2 m vehicle model and speeds up
to 40 m/s. Unsupported dimensions/speeds and nonintegral tick horizons fail
explicitly. CutInEnv bypasses stock random traffic, RL reward calculations, and
multi-frame action scheduling. No controller framework or backend registry is
added. Gym physical action/observation spaces are intentionally not normalized
RL spaces. Direct Gym callers use joint dictionaries and the fixed native dt;
the adapter can supply the runner's dt, which must stay fixed within an episode.

Coordinates use SI units: x follows the road, lane centres are
`lane * lane_width_m`, and heading is radians from positive x. Positions are
vehicle centres. `start_time_s` begins the lateral reference at the next control
tick; PREPARE is deferred. Requested lane-change duration is not guaranteed
physical completion time.
