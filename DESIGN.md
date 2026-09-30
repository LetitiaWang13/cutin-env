# Cut-in MVP Design

Status: revised proposal; no implementation is implied.
Confirmed: target only ego-involving collisions, and use completed-run replay
for the first web version. Removing PREPARE remains open for discussion.

This is a fresh, incremental implementation. `../cutin-v0/` is a reference for
lessons, algorithms, and tests, not a dependency or a feature checklist. This
repository currently uses the directory name `cutin/`; the Python import name
will also be `cutin`.

## 1. Goal and delivery boundary

The eventual product is a web-based scenario exploration tool: users adjust
parameters or ranges, generate/run scenarios, view trajectories and metrics,
and optionally search for safety violations. Scenario configuration is the
shared, simulator-independent description throughout that workflow.

**MVP-A: one headless vertical slice**

- One straight road, exactly three lanes and three vehicles: `ego`, `front`,
  and `cut_in`. Ego/front start in lane 1; cut-in starts in lane 0 and merges
  into lane 1. Lane 2 is empty.
- A validated JSON scenario runs through a highway-env adapter and produces a
  normalized simulation log, collision/gap/TTC metrics, and saved configuration.
- Ego/front use a fixed, documented baseline policy. Cut-in uses a timed,
  physically executed lane change, not a position teleport.
- One benign example, one intentionally unsafe example, and repeatability tests.
- No web frontend or optimizer is required for this first deliverable.

**MVP-B: minimal generation and falsification**

- Seeded uniform sampling over declared initial-condition/cut-in ranges.
- Sequential random search using the same simulator and evaluator as MVP-A.
- Save the best eligible scenario, its metrics, and a small search summary.

**Later:** local web inputs and trajectory replay, adaptive optimizers, comfort
metrics, richer controllers, and additional simulator/export integrations.

Not in the MVP: live pause/step sessions, parallel jobs, databases, plugin
registries, full OpenSCENARIO schemas, Pygame applications, CARLA/SUMO adapters,
advanced artifact management, or guarantees of real-world safety.

## 2. Architecture: contracts, not services

```text
CLI / future web form
        |
        v
ScenarioConfig --------------------------+
        |                                |
        v                                |
Simulation + SimulatorConfig             |
        | uses HighwayAdapter            |
        v                                |
SimulationLog (includes resolved config)  |
        |                                |
        v                                |
Evaluator + EvaluationConfig             |
        |                                |
        v                                |
Metrics -----------------> search loop --+
                              ^
                      Generator + SearchSpace
```

The application coordinates these calls and optionally persists their outputs.
The optimizer calls generation, simulation, and evaluation **in process**; it
does not call its own HTTP API. A future API is an outer transport boundary, not
an additional hop between each internal module.

| Component | Owns | Must not depend on |
| --- | --- | --- |
| Core contracts | Configuration, normalized state/log/metric types, validation | highway-env, HTTP, optimizer implementations |
| Generation | Proposing concrete valid scenarios from a base and ranges | Simulator objects, web state |
| Falsification | Budget, candidate execution, objective, best candidate | highway-env internals, UI |
| Simulation | Per-run lifecycle, fixed clock, controller scheduling, termination | Metrics, search, HTTP |
| Highway adapter | Road/vehicles, physical stepping, normalized state, collision events | Search objective, persistence, UI |
| Evaluator | Pure computation from normalized logs | Simulator objects, driving policies |
| Application | Compose run/search workflows and save results | Backend internals |
| CLI / future API | Parse requests and expose application results | Physics and metric formulas |

This separation does not require microservices, async execution, or a framework.
Only adapter modules import highway-env. External arrays/objects are converted
into our own immutable typed values at that boundary.

## 3. Configuration boundaries

### 3.1 ScenarioConfig: a concrete experiment, not a search space

A `ScenarioConfig` describes **what happens**, with all values resolved. It
contains no slider bounds, optimizer state, backend class names, output paths,
render FPS, or evaluation thresholds.

Proposed v1 fields (all subrecords are immutable dataclasses):

| Field | Meaning |
| --- | --- |
| `schema_version` | `1`; unsupported versions fail explicitly |
| `seed` | Scenario stochasticity; fixed for deterministic MVP runs |
| `road` | `lane_count=3`, lane width and road length in metres |
| `duration_s` | Experiment horizon; independent of wall-clock time |
| `initial.ego/front/cut_in` | Vehicle-centre `x_m`, `lane`, and `speed_mps` |
| `vehicles` | Shared vehicle length and width in metres |
| `baseline_policy` | Fixed ego/front following targets and policy parameters |
| `cut_in` | `start_time_s`, `target_speed_mps`, `lane_change_duration_s` |

Fixed topology and lane assignments are validated, not silently inferred from
highway-env defaults. The baseline policy parameters are recorded so the system
under test is explicit, but are **not searched** in the MVP. Specific gains,
limits, and numerical defaults will be selected in an integration experiment;
this document does not claim they are calibrated.

Proposed simplification relative to v0 (pending confirmation): there is **no
PREPARE phase or merge-time relative position target**. Cut-in timing starts the lateral reference immediately at the
next control tick. Initial relative position is determined by initial x values;
it is not a promised gap at merge initiation. Requested lane-change duration
controls a reference trajectory, not guaranteed completion. Log actual merge
start/completion, or an incomplete maneuver.

Cut-in tracks its target speed before/during merging and follows the baseline
policy after completion. It does not veto an unsafe merge, but respects physical
actuator limits. Ego/front keep their lanes and use a simple bounded following
policy that can react to partial lateral intrusion. Policy/model identifiers
and implementation versions are recorded with the run.

#### Comparing the old and proposed maneuver

| Aspect | v0 | Proposed MVP |
| --- | --- | --- |
| Phases | WAIT -> PREPARE -> MERGE -> FOLLOW | WAIT -> MERGE -> FOLLOW |
| Trigger time | Start preparing relative position | Start lateral reference at the next control tick |
| Position parameter | Desired offset relative to ego at merge start | Initial x positions at t=0 |
| Preparation | Adjust longitudinal speed until target tolerance or timeout | No relative-position feedback |
| Merge-time offset | Approximately targeted, but not guaranteed | Emerges from initial conditions and vehicle motion |

Example: in v0, a trigger at 2 s, target offset +10 m, and preparation timeout
2 s can start merging between 2 s and 4 s. A timeout allows merging even when
+10 m was not achieved. In the proposal, an initial offset +10 m and a start
time of 2 s start merging near 2 s; the offset at that moment may differ.

The old approach supports experiments targeting an approximate merge-time gap.
The proposal instead searches initial conditions and a timed maneuver, with
fewer coupled parameters and simpler timing semantics. Both use physical lane
change dynamics; neither guarantees collision or exact merge completion time.
If merge-time gap targeting is required, retain PREPARE explicitly rather than
hiding position adjustment in the generator or adapter.

### 3.2 Other configurations

- `SimulatorConfig`: **how it runs** — backend identifier (`highway` only),
  physics/control rates, adapter-specific numerical/model settings. No scenario
  values are secretly overridden. Unsupported scenario features fail clearly.
- `EvaluationConfig`: **how it is judged** — metric version and optional safety
  thresholds. Changing it permits re-evaluation without changing the scenario.
- `SearchSpace`: a base `ScenarioConfig` plus explicit ranges over an allowlist
  of numeric paths. Initially: vehicle x/speeds and the three cut-in parameters.
  Road topology, baseline policy, seeds, and backend settings are not searched.
- `SearchConfig`: candidate budget, attempt cap, and separate search RNG seed.
  Objective v1 is collision-first ranking as defined below.

A web form for a single run creates `ScenarioConfig`. A form for search creates
`SearchSpace` and `SearchConfig`; it must not pretend a range is a concrete value.
Generation resolves ranges into the same `ScenarioConfig` used by direct runs.

### 3.3 Validation and serialization

Use standard-library dataclasses, explicit validation, and JSON conversion;
do not add a generic schema framework initially. Type annotations alone do not
validate user input. Reject unknown fields, booleans as numbers, non-finite
values, negative speeds/times, invalid lane assignments, initial footprint
overlaps, vehicles outside the road, and front initially behind ego.

Require positive lane-change duration/rates and a control rate dividing the
physics rate. Duration must span an integral number of physics ticks. Reject
unsupported dimensions/model values rather than silently ignoring them. A
cut-in that cannot finish within the horizon is allowed but reported incomplete.

Defaults may be accepted on input but must be fully expanded before execution
and saving. Never mutate a base config while sampling. JSON must use finite
numbers and `null` for unavailable values, never NaN or infinity. Each top-level
persisted contract has `schema_version`; v1 has no v0 compatibility requirement.

## 4. Normalized data contracts

### Coordinates and sampling

- SI units; x increases down the road, y increases from lane 0 toward lane 2.
- Lane centres: `y = lane_index * lane_width_m`.
- Heading is radians from positive x. Vehicle positions refer to centres.
- Stable roles identify vehicles; list order is not identity.
- Log the initial state at t=0 and every physics step, including the terminal
  state. Timestamps derive from integer tick count, not accumulated wall time.

### Minimal types

- `VehicleState`: role, x/y, heading, speed, length/width, collision flag.
- `Snapshot`: timestamp and immutable tuple of the three vehicle states.
- `ControlAction`: desired longitudinal acceleration and steering angle.
- `LogSample`: snapshot and the actually applied, bounded actions for the
  interval ending at that snapshot (none for the initial sample).
- `SimulationEvent`: time, event kind, involved roles; initially maneuver
  start/completion and collision. Collision flags remain available even when
  the backend cannot reliably identify a collision pair.
- `SimulationLog`: resolved scenario/simulator configs, samples, events,
  termination, actual end time, and runtime/model provenance.
- `Metrics`: collision information, pair-specific minimum gap/TTC with times,
  and assessment validity. It references the evaluated run and evaluator version.
- `RunResult`: application-level bundle of log and metrics; not the simulator's
  return type and not a replacement for `ScenarioConfig`.
- `SearchResult`: best config/metrics (nullable), attempt/completion/error counts,
  objective version, and references to the winning run.

Proposed terminations: `horizon`, `collision`, `road_exit`, `error`, `cancelled`.
Errors/cancellation retain available partial logs and diagnostics. They must not
be mislabeled as successful safe runs; input validation errors occur before a
run exists. Road exit is an incomplete assessment, not a safety success.

Initially logs fit in memory; JSON is sufficient. Add JSONL streaming only when
run size creates a real need. The evaluator must not read files implicitly.

## 5. Python interfaces and execution

Signatures below express contracts, not a requirement to introduce base classes:

```python
# Public module-level operations
simulate(scenario: ScenarioConfig, simulator: SimulatorConfig) -> SimulationLog
evaluate(log: SimulationLog, config: EvaluationConfig) -> Metrics
sample(base: ScenarioConfig, space: SearchSpace, rng: Random) -> ScenarioConfig

# Application composition
run_scenario(scenario, simulator_config, evaluation_config) -> RunResult
search(space, search_config, simulate_fn, evaluate_fn) -> SearchResult
```

For search, bind simulator/evaluator configs to the injected callables so the
loop consumes only `ScenarioConfig -> SimulationLog -> Metrics`. Functions are
sufficient for random sampling and random search; a stateful proposal/feedback
interface can be added when an adaptive optimizer actually needs one.

The minimal internal adapter seam is:

```python
reset(scenario, simulator_config) -> Snapshot
step(actions_by_role, dt_s) -> Snapshot
close() -> None
```

The adapter may expose normalized events and road-exit status alongside each
snapshot. Only define a small `Protocol` if it helps testing; no registration
system, inheritance tree, or automatic backend discovery is needed.

The simulation owns tick count, duration, scheduling, and stop policy. At each
control tick, all three controllers read the **same** snapshot; actions are
computed before any physical update, bounded, then held between control ticks.
The adapter owns physics and collision detection. Every run gets fresh adapter
and controller state; cleanup runs even on errors.

Start with highway-env road/vehicle primitives or the smallest necessary custom
environment, not stock randomized traffic or Gym reward. Verify that internal
actions cannot overwrite our controls. Rendering is not required for simulation.
Do not create a custom controller framework: a small `reset`/`act` contract and
ordinary functions/classes are sufficient.

## 6. Evaluation and falsification semantics

### Metrics v1

Compute safety metrics only from normalized samples/events, not Gym rewards:

1. **Collision:** backend collision flags/events; distinguish ego involvement
   from collisions only between the other vehicles. Objective v1 targets ego
   collision, while reporting all collisions.
2. **Signed longitudinal gap:** for ego versus each other vehicle when world-axis
   lateral footprint bounds overlap, use absolute x-centre separation minus
   projected longitudinal half-lengths. Negative gap is not proof of collision.
3. **Positive longitudinal TTC:** for the same pairs, identify the trailing
   vehicle and compute positive gap / positive closing speed using x-projected
   velocity. Non-closing, overlapping, or laterally disjoint cases yield `null`.

Use heading-aware rectangle projections for conservative footprint bounds.
Minima aggregate only applicable samples. TTC is a constant-velocity local
approximation, not a prediction of future lateral motion; it can miss a pending
cut-in. Sampled metrics can miss between-step extremes. Document these limits.

Only horizon runs and runs ending in an ego-involving collision are eligible
for ranking. A cut-in/front collision without ego involvement is a non-target
collision, not a falsification success. Since execution stops at the first
collision, exclude that truncated run from ranking rather than label it safe
or reward its small gap. Report non-target collisions separately. Simultaneous
collisions involving ego remain target violations. Use backend collision
evidence, not negative geometric gap, to determine ego involvement.

An incomplete run may retain observations, but those observations do not establish
safety. Even a completed collision-free run is not a proof of safety.

### Search v1

- Freeze base scenario, simulator, evaluator, and baseline policy for a search.
- Sample independently and uniformly over declared ranges with a local seeded
  RNG and stable parameter order. Scenario seed stays fixed to isolate changes.
- Validate every sample; rejected samples count toward a finite attempt cap.
- Budget counts completed eligible runs; errors, road exits, and non-target
  collisions count toward the attempt cap, not as unsafe discoveries. Stop at
  either bound; summarize non-target collisions separately.
- Rank lexicographically: ego collision first, then lower minimum ego-pair gap,
  then lower positive ego-pair TTC. Missing values rank after applicable values;
  stable ties keep the earlier candidate. No invented composite safety score.
- Save only the best full run initially; summarize failures/rejections. If no
  eligible candidate exists, return `best_config: null` explicitly.

Random search is an empirical falsification baseline. Future optimization can
replace proposal/ranking logic without altering simulation or evaluation.

## 7. Repository and package layout

Use `pyproject.toml` with a standard build backend, a `src/` layout, and `uv` for
dependencies, lockfile, execution, and checks. Ship an installable Python package
and thin `cutin` CLI. Use only FOSS libraries without telemetry/hosted services;
pytest and Ruff are development dependencies. highway-env remains isolated to
its adapter, though the first package may install it as a normal dependency.

```text
pyproject.toml
uv.lock
README.md
DESIGN.md
src/cutin/
  __init__.py               # Package metadata; no eager backend imports
  core/
    __init__.py
    scenario.py             # ScenarioConfig and validation
    state.py                # Snapshot, VehicleState, ControlAction
    log.py                  # SimulationLog, events, termination, provenance
    metrics.py              # Shared Metrics contract, not metric algorithms
  simulation/
    __init__.py             # Public simulate() entry point
    config.py               # SimulatorConfig
    runner.py               # Lifecycle, clock, controller scheduling
    controllers.py          # Fixed following and timed cut-in policies
    adapters/
      __init__.py
      highway.py            # Only highway-env integration
  evaluation/
    __init__.py             # Public evaluate() entry point
    config.py               # EvaluationConfig
    safety.py               # Collision, gap, TTC, geometry helpers
  generation/               # MVP-B only
    __init__.py             # Public sample() entry point
    space.py                # SearchSpace and allowed ranges
    sampling.py             # Seeded uniform sampling
  falsification/            # MVP-B only
    __init__.py             # Public search() entry point
    config.py               # SearchConfig
    objective.py            # Eligibility and ego-focused ranking
    search.py               # Budgeted loop and SearchResult
  application/
    __init__.py             # Public run_scenario()/run_search() workflows
    workflows.py            # Component composition and RunResult
    storage.py              # Explicit artifact loading/saving
  interfaces/
    __init__.py
    cli.py                  # Thin CLI using application workflows
    web/                    # Later only: API wrapper and static frontend
examples/
  baseline.json
  unsafe.json
  search.json              # MVP-B only
tests/
  core/                    # Contract validation and serialization
  simulation/              # Policies, adapter, clock, headless execution
  evaluation/              # Synthetic logs and metric edge cases
  generation/              # MVP-B only
  falsification/           # MVP-B only; fake simulator/evaluator callables
  integration/             # End-to-end workflows and packaging
```

Directories express component boundaries, not separate deployable services or
separate distributions. Start with one installable package; independent packages
can be considered only when there is a concrete distribution need. Each
component has a small public API in its `__init__.py`; callers must not import
another component's implementation files. Do not create deferred empty modules.

Configuration belongs to its owning component: ScenarioConfig in core,
SimulatorConfig in simulation, EvaluationConfig in evaluation, SearchSpace in
generation, and SearchConfig in falsification. Shared state/log/metric contracts
belong to core so neither evaluation nor simulation must import the other.
The SimulationLog contains a resolved simulator-settings JSON record; it must
not import the concrete SimulatorConfig type into core. Component validators
produce this record, preserving backend-independent serialization.

Allowed dependency direction:

```text
interfaces -> application -> simulation / evaluation / generation / falsification
                              |            |            |             |
                              +------------+------------+-------------+-> core
falsification -> generation's public sampling API
simulation.runner -> simulation.controllers / simulation.adapters
```

Core imports no other component. Simulation does not import evaluation;
evaluation does not import simulation. Falsification uses injected simulation
and evaluation callables, not their concrete implementations. Only application
selects and binds components. Interfaces contain no control, search, or metric
algorithms. Controllers are simulation-internal for now, not a separate plugin
system. Add import-boundary checks alongside packaging tests when implementing.

A run directory initially needs only `scenario.json`, `simulator.json`,
`evaluation.json`, `log.json`, and `metrics.json`. The log includes resolved
configs so it can be evaluated on its own; separate files are convenient views
of those same values, not independently editable sources of truth. Output
locations belong to the caller; never silently overwrite an existing run.

CLI proposal: `uv run cutin run examples/baseline.json --output runs/baseline`.
Simulator/evaluator settings can initially use explicit recorded defaults.
Library callers do not have to write artifacts. Include a wheel build/install
smoke check so packaging works outside the repository root.

## 8. Future web/API boundary

Confirmed: start with completed-run replay, not live simulation sessions.
Browser playback pause/seek changes only the view; backend live step/control is
deferred. The first web version belongs under `interfaces/web/`:


- `POST /runs`: validated scenario plus simulator/evaluator settings; invokes
  `run_scenario` and returns run id, metrics, and log/artifact references.
- `GET /runs/{id}` and `/runs/{id}/log`: retrieve result and normalized samples.
- Later `POST /searches`: base scenario, ranges, budget, and seeds.

These are proposed payload semantics, not endpoints to implement now. Long
searches will need a job boundary, but job queues/authentication/multi-user
storage are deferred. Even a local API must validate inputs and restrict origin,
binding, and request size before accepting mutations.

An HTML/Canvas frontend can replay the normalized trajectory and show metrics;
it must not implement vehicle physics. Rendering cadence cannot affect simulation
timing. Backend replacement should preserve these data contracts, not renderer
internals. OpenSCENARIO/OpenDRIVE are interchange formats rather than simulators;
a future exporter may translate our limited scenario subset without making our
core schema a copy of those standards.

## 9. Five-person work split and integration order

Contracts are reviewed jointly before parallel implementation. One owner merges
contract changes; no contributor silently changes shared field meanings.

| Owner | Main files | First testable deliverable |
| --- | --- | --- |
| 1: contracts/application | core/, application/, package setup | Validated JSON round-trip and fake-backed workflow |
| 2: simulation integration | simulation/runner.py, config.py, adapters/ | Exactly three vehicles, fixed stepping, normalized log |
| 3: policies/control | simulation/controllers.py | Unit-tested following, lane keeping, timed cut-in and reset |
| 4: evaluation | evaluation/ | Synthetic-log collision/gap/TTC tests including null cases |
| 5: generation/tooling | interfaces/cli.py, then generation/ and falsification/ | CLI on fake workflow, seeded sampler and fake-backed search |

Owner 5 can build against fake callables before highway integration is ready;
owner 4 can use synthetic logs. Owner 1 and owner 2 agree lifecycle/result/error
behavior first. Web work begins only after the headless interface is stable;
it need not be a sixth concurrent subsystem.

Delivery sequence:

1. Agree contract fields, parameter semantics, and small synthetic fixtures.
2. Package/config/log serialization plus a highway integration spike.
3. One actual scenario -> normalized log -> metrics -> saved artifacts (MVP-A).
4. Seeded sampling and a bounded search using unchanged contracts (MVP-B).
5. Optional browser form and completed-run replay.

Work on separate `feat/` branches with small commits. Review algorithms and
contract changes separately from structural refactoring. Reuse v0 knowledge,
but do not copy its session/UI/search machinery wholesale.

## 10. Acceptance and open decisions

MVP-A acceptance:

- Fresh `uv` install and packaged import/CLI both work.
- Validated configs round-trip; malformed inputs fail before simulation.
- Exactly three vehicles and fixed lane assignments; controls survive physical
  stepping, lane changes use dynamics, and terminal samples are present.
- Simulation works headlessly without opening a window.
- Same configs/seeds in the same dependency/platform environment reproduce
  traces/metrics within declared numeric tolerances; no cross-platform bitwise
  guarantee is implied.
- Changing cut-in timing, speed, duration, or initial x changes executed motion.
- Benign and unsafe fixtures exercise meaningful metrics; synthetic tests cover
  collision, partial intrusion, non-closing pairs, null minima, and errors.
- Evaluation can run on a deserialized log without constructing highway-env.

MVP-B additionally demonstrates bounded attempts, deterministic sampling,
unchanged unsearched fields, errors excluded from ranking, and a saved winning
config that reproduces the result. No eligible result is handled explicitly.

Team decisions to confirm before implementation:

1. Is headless MVP-A followed by search MVP-B the desired delivery sequence?
2. Can we omit v0's merge-time target-position/PREPARE behavior and optimize
   initial position plus timed merging instead?
3. Which fixed ego/front policy and vehicle dimensions should the integration
   spike establish? Freeze and record them before comparing search results.

Confirmed decisions:

- Only collisions involving ego are falsification successes; cut-in/front-only
  collisions are reported separately and excluded from ranking.
- The first web demonstration uses completed-run Canvas replay. Live backend
  pause/step control is deferred.
- Organize code into component subpackages with explicit public APIs and
  dependency direction, rather than flat top-level implementation modules.

> Recap: Build the smallest headless config-to-log-to-metrics slice first. Keep concrete scenarios, search ranges, simulator settings, and evaluation settings separate; add random search and web replay through the same contracts only after that slice is stable.
