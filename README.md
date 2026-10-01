# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for delivery order.

## Current scope

The runner executes configured car-following, lane keeping, and cut-in policies
through CutInEnv(HighwayEnv), with optional native visualization. An independent
evaluator computes collision, gap, and TTC from logs. Thin application workflows,
a packaged CLI, explicit run artifacts, and seeded scenario sampling are available.
Bounded sequential random falsification and a local Web form/metrics/replay UI
are implemented. No collision-free run or search is a proof of safety. Numerical defaults and warning
thresholds are experimental, not safety-calibrated.
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

## Local Web MVP

```sh
uv sync --locked
uv run cutin web --port 8000 --runs-directory runs
```

Open **http://127.0.0.1:8000/** in a modern desktop browser.

1. Choose Baseline or Unsafe regression, or edit initial positions/speeds,
   merge timing/target, seed, horizon, numerical rates, and warning thresholds.
2. Click **Run and save**. The existing headless application workflow computes
   the complete run before returning; metrics appear beside the replay.
3. Use Play/Pause, the tick slider, and playback speed to inspect recorded
   snapshots. These change only the browser view, never physics or controls.
4. Load saved run directories or search winners from the dropdown. For example,
   runs/search-001/best is exposed as search-001 when serving the runs directory.
   Loading/refreshing does not re-simulate or re-evaluate anything.

Web runs save under runs/<random-id>/ in the existing five-file format. Metrics
show assessment/termination, ego collision, warnings, pair minima and their ticks;
raw metrics, simulator provenance, and diagnostics are expandable. An error or
empty partial log remains incomplete, not safe. The Canvas draws stored poses
and collision flags; it is not the native Gym viewer or a browser simulator.
CLI native rendering remains available separately.

The standard-library server is **local development only**, not a production
HTTP service. It binds only 127.0.0.1, checks Host/Origin and mutation headers,
accepts JSON requests up to 64 KiB, bounds Web runs to 60 s/120 physics Hz, and
limits each replay artifact to 16 MiB. One simulation runs at a time; another
mutation gets 409 rather than being queued. Artifact IDs cannot select arbitrary
paths or escape the configured root. No telemetry/CDN/framework dependencies.

There is no live backend pause/step, Web search job endpoint, authentication,
remote binding, session manager, or typed log re-import. Search still runs via
CLI; its winner can be replayed here. Stop the server with Ctrl+C. Persistence
retains the existing non-crash-atomic limitations. Treat the artifact root as
trusted local data, not a multi-user upload store.

Optional developer browser smoke (Playwright is not a runtime dependency):

```sh
uv run --with playwright playwright install chromium
uv run --with playwright python scripts/smoke_web_browser.py
```

Linux may also need Chromium OS libraries. PLAYWRIGHT_CHROMIUM_EXECUTABLE can
select an existing browser; CUTIN_SMOKE_SCREENSHOT optionally saves a screenshot.
HTTP/security/real-run tests run in pytest; JavaScript selection/null tests use
Node when available. Chromium form/metrics/playback/persistence were also verified.

## Run and save an experiment

```sh
uv sync --locked
uv run cutin run examples/baseline.toml
uv run cutin run examples/unsafe.toml --render --output runs/unsafe-001
uv run cutin run examples/baseline.toml --gap-warning-m 4 --ttc-warning-s 2
```

Close the terminal scene's window to see metrics in the terminal. `--output`
creates a **new directory** containing:

```text
scenario.json    validated scenario
simulator.json   requested numerical/controller settings
evaluation.json  warning thresholds
log.json         sampled state/actions, termination, backend metadata
metrics.json     assessment, collision evidence, minima/ticks, warning settings
```

Existing files/directories/symlinks are never overwritten. No output path means
no artifact writes. In-memory library use stays independent of persistence:

```python
from pathlib import Path
from cutin.application import load_scenario, run_scenario, save_run

result = run_scenario(load_scenario(Path("examples/baseline.toml")))
print(result.metrics)
save_run(result, Path("runs/baseline-001"))  # Optional; must not already exist.
```

Saved scenario.json is also validated input for another simulation (not replay):

```sh
uv run cutin run runs/unsafe-001/scenario.json --output runs/unsafe-002
```

A collision is an experiment result, not a process failure. Simulation errors
print diagnostics, save partial log/metrics when requested, and exit 1. Invalid
requests and I/O failures exit 2. Write failures clean up a newly created run
directory; process crashes are not guaranteed atomic. Typed log/metrics reload and artifact indexes remain deferred; the Web consumer
reads raw saved JSON explicitly for replay.

## Legacy scenario script

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
actual cut-in reference-start/completion ticks, and safety metrics. See native
visualization below. `--output` still exports the raw log, not a log/metrics bundle.

`simulate(scenario, SimulatorConfig())` executes policies by default. Supplying
explicit `actions=...` selects fixed-action diagnostic mode instead.

The legacy script shares the application workflow and native viewer with the
CLI; its `--output` still means a **single raw log file**, not a run directory.

## Search for ego-involving collisions

```sh
uv run cutin search examples/baseline.toml --space examples/search_space.toml \
  --budget 5 --max-attempts 10 --seed 17 --output runs/search-001
# Re-simulate the best concrete scenario with the native viewer (not log replay).
uv run cutin run runs/search-001/best/scenario.json --render
```

The locked example completes five eligible runs and finds two ego collisions;
proposal 3 wins. The separate scenario seed stays 42. Search remains headless;
visualize the winner afterward, not every attempt.

- `--budget` counts eligible completed assessments, not total attempts.
- `--max-attempts` counts everything: valid runs, rejected candidates, exceptions,
  backend errors, road exits, non-target collisions, and incomplete assessments.
  The search stops at either bound; attempt cap may be smaller than the budget.
- Ranking is lexicographic: ego collision first, then lower minimum ego-pair gap,
  then lower positive TTC. Missing values sort last and exact ties keep the
  earlier proposal. Warning thresholds do not affect ranking.
- Finding a target collision does not stop the loop; the remaining budget can
  refine the best run. Cancellation stops immediately and preserves an earlier
  best result. KeyboardInterrupt propagates; no resumable session is promised.
- Exceptions are counted and the last diagnostic is retained. All attempts have
  one outcome counter; ego_collisions is a subset of completed, not extra attempts.

Optional output contains base.json, space.json, simulator.json, evaluation.json,
and **search.json** (bounds/seed, counts, stop reason, best proposal/config/metrics,
last error/rejection). **best/** contains only the winning run's five artifacts.
There are no per-candidate full logs. If no eligible candidate exists,
`best_config`, `best_metrics`, and `best_run` are null, with no best directory.
Existing output paths are refused. No output means no persistence. A search
with no winner and operational errors saves its summary then exits 1; reaching
an attempt cap through rejected/non-target candidates is a valid result, not a
safe result. Invalid input/storage errors exit 2.

Library callers can use application.search_scenarios/save_search, or inject
one-argument simulate/evaluate callables into falsification.search. The search
component owns eligibility, counters, and ranking; application only binds fixed
simulator/evaluation settings and saves results when explicitly asked. Bounds
are run-count limits, not a wall-clock timeout.

## Sample one concrete scenario

```sh
uv run python scripts/sample_scenario.py --seed 17 --output runs/candidate-001.json
uv run cutin run runs/candidate-001.json --render --output runs/candidate-001
```

The first command reads examples/baseline.toml and examples/search_space.toml.
Use `--space path.toml` or a positional base TOML/JSON to change inputs. Without
`--output`, it prints concrete scenario JSON to stdout. Existing output files
are refused. This diagnostic samples once, without physics or automatic retries;
a rejected candidate exits 1, invalid configuration/I/O exits 2.

The range file uses explicit paths:

```toml
schema_version = 1
[ranges]
"initial.cut_in.x_m" = [45.0, 80.0]
"cut_in.start_time_s" = [0.0, 3.0]
"cut_in.lane_change_duration_s" = [0.8, 3.0]
```

Library callers own the local RNG and its advancing stream:

```python
from random import Random
from cutin.generation import ParameterRange, RejectedSample, SearchSpace, sample

space = SearchSpace((ParameterRange("initial.cut_in.x_m", 45, 80),))
rng = Random(17)
try:
    candidate = sample(base, space, rng)  # base is a validated ScenarioConfig.
except RejectedSample as error:
    print("Count this rejected attempt:", error)  # No retry inside sample().
```

Only initial x/speeds for ego/front/cut-in and the three cut-in parameters are
searchable. x is the world position at tick 0, not an offset at merge start.
Road, lane assignments, baseline policy, duration, and scenario seed stay fixed.
Proposal seed 17 is separate from the unchanged scenario seed (42 in examples).
Path ordering does not change proposals; fixed bounds consume no random draw.
Ranges have finite ordered bounds/span, but are not pre-certified feasible:
invalid geometry/speeds/times are rejected after the draw, never clamped. A valid
scenario may still exceed backend-specific capabilities, which the simulator
reports. A merge beyond the horizon is permitted, not rejected by generation.
The same generated ScenarioConfig feeds existing simulation/evaluation/storage.

## Evaluate safety metrics

```python
from dataclasses import asdict
from cutin.evaluation import EvaluationConfig, evaluate

# log comes from simulate(); evaluation never runs physics or reads files.
metrics = evaluate(log, EvaluationConfig(gap_warning_m=2, ttc_warning_s=1.5))
print(metrics.assessment_status, metrics.ego_collision)
print(asdict(metrics))  # JSON-compatible; unavailable quantities are None/null.
```

- Metrics report ego/front and ego/cut-in minimum signed longitudinal gap and
  positive longitudinal TTC, with their observed integer ticks. Convert ticks
  to seconds using `metrics.physics_hz`; exact ties keep the first sample.
- Heading-aware world-axis footprint bounds determine lateral applicability.
  No lateral overlap means no applicable gap/TTC. Zero/negative gap or
  non-closing motion means no positive TTC, not TTC=0 or infinity.
- Backend collision flags are authoritative; negative gap is not collision proof.
  First collision roles indicate flagged participants, not exact collision pairs.
- `complete`: horizon reached or an ego-involving collision ended the run.
  `non_target_collision`: only other vehicles collided and stopped execution.
  `incomplete`: road exit, error, cancellation, or missing ego observations.
  Only complete assessments can
  enter later search ranking; complete does not mean safe.
- Strictly-below threshold warnings are summaries, not the falsification objective.
  Threshold changes never change raw minima; partial warnings remain partial.
- Evaluation requires resolved physics_hz metadata, consecutive samples from
  tick 0, finite valid state values, and consistent termination evidence.
  Malformed logs raise ValueError rather than becoming successful assessments.
- For multiple vehicles sharing a role, minima aggregate all ego/other-role
  combinations. No persistent vehicle identity is inferred from snapshot order.

Baseline has no collision and minimum ego/cut-in gap about 12.797 m. Unsafe has
an ego/cut-in collision at tick 18 (0.6 s) and negative gap. Both have no applicable
positive TTC: the unsafe lateral merge intrudes into an already-overlapping
longitudinal footprint. TTC is a local constant-velocity approximation, not a
prediction of lateral motion. Conservative bounds and sampling can overstate
proximity or miss between-step extremes; no run establishes controller safety.

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

- `interfaces/web/server.py`: local HTTP boundary, resource/origin/root limits,
  serialized headless runs, and explicit persisted-document reads.
- `interfaces/web/assets/`: self-hosted HTML/CSS/JavaScript form and recorded-pose
  Canvas replay, without physics or metric formulas.
- `falsification/config.py`: eligible-run budget, attempt cap, and proposal seed.
- `falsification/objective.py`: target eligibility and lexicographic ranking.
- `falsification/search.py`: bounded loop, diagnostics/counters, and best full run;
  no backend, application, or file-I/O dependency.
- `generation/space.py`: explicit immutable numeric ranges and a nine-path allowlist.
- `generation/sampling.py`: one local-RNG uniform proposal, candidate validation,
  and explicit rejection; no backend, file I/O, or internal retry loop.
- `application/workflows.py`: compose simulate/evaluate into an in-memory
  RunResult with requested settings; callable injection keeps tests backend-free.
- `application/storage.py`: explicit validated scenario loading and fresh-directory
  artifact saving, outside physics/evaluation. No implicit reads or writes.
- `interfaces/cli.py`: request parsing, native viewer lifecycle, and terminal
  summaries; no metric formulas or physics loop.
- `core/metrics.py`: immutable result contracts and assessment status, without
  metric algorithms. `core/geometry.py` shares footprint conventions between
  evaluation, controllers, and road-bound checks.
- `evaluation/config.py`: experimental warning thresholds, validated at evaluation.
- `evaluation/safety.py`: pure offline evaluate(log, config), backend-independent.
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
