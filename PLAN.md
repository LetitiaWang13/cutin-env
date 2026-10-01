# Implementation Plan

Implement testable vertical slices, not empty scaffolding for every future module.
DESIGN.md defines the component boundaries and scenario semantics.

## 1. Package foundation and minimal contracts — complete

- Set up an installable `src/cutin` package, uv lockfile, pytest, and Ruff.
- Implement ScenarioConfig with boundary-only validation, TOML input, and plain
  JSON-compatible output; no constructor validation hooks.
- Implement small immutable state/action/log records with integer snapshot ticks.
- Test parsing/round trips, malformed external inputs, and installed imports.
- No simulator dependency, CLI, controllers, metrics algorithms, or search yet.

Acceptance: a scenario can be loaded into a validated ScenarioConfig; the wheel
can be imported outside the repository. Runtime defaults are illustrative, not
calibrated. State/log contracts may be refined by the backend integration spike.

Verified for step 1:

- `uv sync --locked`, 58 passing pytest cases, Ruff lint/format checks.
- Wheel and source distribution build successfully with `uv build`.
- The wheel installs/imports in an isolated uv environment outside the repository.
- The TOML baseline loads and round-trips through JSON-compatible values.
- External input is validated by from_dict(); Python-created/modified scenarios
  use explicit validate_scenario() before execution.
- Actions live separately from state. Internal records have no constructor
  validation, event framework, or generic three-vehicle count constraints.
- StateSnapshot stores a flat tuple. vehicles_for_role() returns every matching
  vehicle as a tuple, or an empty tuple; no mapping wrapper or constructor hook.
- State/log records remain small data contracts. Step 2 adds actual stepping,
  simulator metadata, and a diagnostic JSON export.

## 2. Highway-env integration probe — complete

- Add SimulatorConfig and the highway-env adapter.
- Initialize exactly three vehicles; normalize snapshots and advance fixed steps.
- Check external actions survive backend stepping, collision flags work, and no
  window is opened. Record resolved simulator settings in the log.
- Do not add full driving policies before verifying backend behavior.

Verified for step 2:

- 90 total passing tests: backend layout, shared fixed dt, external actions,
  actual bounded-action logging, speed bounds, deterministic reruns, and headless
  execution without pygame display initialization.
- Collision flags distinguish ego/front from cut-in/front-only collisions.
- `scripts/probe_highway_env.py` prints real initial/final states, optionally saves
  a JSON log with resolved simulator settings, and refuses output overwrites.
- Probe cutoff is cancelled; horizon, collisions, and conservative road exits
  are explicit. This diagnostic does not execute cut-in timing or driving policy.
- No renderer, Gym environment wrapper, abstract backend hierarchy, or general
  persistence framework was added. The adapter counts physical ticks; the future
  runner will schedule controllers from that same clock.

## 3a. Separate translation and execution policy — complete

- Backend naming is `highway-env` in metadata/docs, `highway_env` in Python modules,
  and HighwayEnvAdapter for the class. No ambiguous `highway` backend alias.
- Configuration/object construction, action translation, and state translation
  are small functions separate from adapter lifecycle/physical stepping.
- The adapter accepts dt and has no SimulatorConfig dependency. Generic bounds,
  horizon validation, stop policy, and log assembly belong to the runner.
- The runner supports zero/fixed actions; the probe calls it rather than owning
  a duplicate loop. Fake-adapter tests verify dt, bounds, stop/error behavior,
  partial logs, and cleanup. This slice originally ran fixed actions only;
  the following slice adds policies without changing backend translation.
- Verified: 103 tests pass, including real backend stepping and fake-backed
  execution, plus Ruff and package build/import checks.

## 3b. Controlled simulation — complete

- Add car-following, lane keeping, and timed cut-in (PREPARE remains deferred).
- Extend the existing runner with same-snapshot decisions at controller ticks
  and held actions between those ticks; keep backend translation unchanged.
- Check repeatability and that scenario parameter changes affect actual motion.

Acceptance: ScenarioConfig -> SimulationLog on the real backend.

Verified for step 3b:

- `simulate()` executes driving policies by default; explicit actions retain raw
  probe mode. Controller ticks share one snapshot and hold actions between ticks.
- One small controllers.py holds following, lane tracking, smooth timed merging,
  and a fresh per-run policy. No plugin framework, PREPARE, or event subsystem.
- CutInReport records reference-start and actual completion ticks. Late starts,
  incomplete maneuvers, and policy errors are not hidden as successful merges.
- `scripts/run_scenario.py` runs baseline/unsafe TOML inputs and exports JSON.
- Baseline completes physically without collision; unsafe.toml collides ego with
  cut-in, not front. Timing/speed/duration/initial-position changes affect motion.
- 127 tests pass, covering synthetic policies, fake-backed scheduling, real
  controlled runs, repeatability/isolation, and script exports. Ruff and wheel
  build/install checks pass.

## 3c. Native visualization for developer verification — complete

Inserted before metrics to let developers visually inspect actual driving.

- CutInEnv(HighwayEnv) hosts our scenario with joint physical action spaces,
  numeric observations, zero reward, and exactly one physics step per Gym step.
- Reuse native EnvViewer for human/rgb_array modes. No stock traffic, learning,
  RL action normalization, or inherited multi-step scheduling.
- HighwayEnvAdapter delegates physics/clock to CutInEnv; translation and driving
  policies remain separate. Headless execution does not create a viewer.
- `scripts/run_scenario.py --render` opens the native display, retains terminal
  scenes until close, and records early window close as cancelled.
- A small observer callback keeps rendering outside runner scheduling; keyboard
  driving cannot override external actions. No forms/pause/single-step controls.
- Verified: 142 tests pass; native RGB pixels, rendered/headless log equivalence,
  pre-Gym/wrapped physics equivalence at 20/30 Hz, Gym API checks, close/error
  cleanup, and installed-wheel rendering smoke. Ruff and builds pass.
- Real desktop appearance/input remains a manual smoke check; automated native
  window-event tests use SDL dummy. One native viewer at a time is supported.

## 4a. Independent safety metrics — complete

- Added core Metrics/PairMetrics and explicit complete/non-target/incomplete
  assessment status. Minima retain ticks, not an additional floating clock.
- Shared heading-aware footprint projections in core.geometry without creating
  any evaluator dependency on driving policies or backend objects.
- Implemented offline collision evidence, signed gap, and positive longitudinal
  TTC with null applicability, role-group aggregation, and first exact-tie ticks.
- Added configurable experimental warning thresholds; warnings do not change
  raw minima or determine falsification. No rewards or composite score.
- Validation occurs at evaluation boundaries, not record constructors. Reject
  malformed time/termination/nonfinite-state data instead of classifying it safe.
- Scenario script prints metrics; raw JSON log export remains unchanged. Full
  log/metrics artifact persistence belongs to the next application step.
- Verified: 201 tests pass, including synthetic null/heading/closing/minimum and
  collision cases, real benign/ego/non-target collisions, backend-free imports,
  and scenario-script output. Ruff, build, and isolated wheel evaluation pass.
- Unsafe lateral intrusion has no positive longitudinal TTC; backend flags still
  correctly identify its ego collision. No collision-free run is a safety proof.

## 4b. Application and user entry point — complete

- RunResult bundles log, metrics, and requested simulator/evaluation settings.
  A small workflow composes simulate/evaluate and permits fake-backed tests.
- Explicit loading validates TOML/JSON scenarios. Optional save_run writes five
  JSON views to a new directory, refuses existing destinations, encodes before
  creation, and removes the new directory after write errors.
- Packaged `cutin run` accepts optional native rendering, warning thresholds,
  and artifact output. Metrics remain in the terminal/metrics.json, not viewer UI.
- Operational simulation errors save partial results when requested and exit 1;
  bad requests/storage errors exit 2. Collisions are outcomes, not process errors.
- Legacy script retains raw-log export but delegates composition, native viewer,
  and terminal summaries to the shared workflow/interface functions.
- Verified: 225 tests pass, including fake composition, backend-free application
  imports, artifact consistency/cleanup/no-overwrite, saved JSON input, outside-
  repository CLI execution, and native terminal-close persistence. Ruff, build,
  and isolated installed-wheel CLI/artifact smoke pass.
- No crash-atomic database, artifact index, or typed log reload is added. Completed
  log replay remains the web step; current saved-scenario input re-simulates.

## 5a. Seeded scenario proposals — complete

- Added SearchSpace/ParameterRange with explicit allowlisted initial x/speed and
  cut-in ranges, immutable tuples, schema version, and format-independent parsing.
- sample(base, space, rng) draws once in stable alphabetical path order, leaves
  base/scenario seed and global RNG untouched, and skips draws for fixed bounds.
- Validate configurations before RNG use; validate candidates through the core
  scenario boundary. RejectedSample is explicit and never triggers hidden retry.
- Proposal syntax and scenario feasibility stay distinct. No clamping, backend
  dependency, relative-position conversion, or invented completion requirement.
- Added examples/search_space.toml and scripts/sample_scenario.py to export a
  concrete JSON scenario that the existing packaged run CLI accepts unchanged.
- Verified: 274 tests pass, including deterministic streams/order/zero-draw cases,
  all nine fields, input validation, single-draw rejection, immutable base/global
  RNG isolation, backend-free imports, and real sampled scenario execution.
  Ruff, builds, and isolated installed-wheel sampling smoke pass.

## 5b. Bounded falsification — complete

- Added positive budget/attempt cap and separate nonnegative proposal seed.
  Count only eligible assessments toward budget; every attempt has one outcome.
- Implemented collision-first/gap/TTC lexicographic ranking with missing values
  last and stable ties. No composite score, warning-driven objective, or rewards.
- Backend/evaluator exceptions, errors, road exits, non-target collisions, and
  incomplete assessments never win. Cancellation stops before evaluation.
- Keep only the best full log/metrics, retain counts and last diagnostics, and
  return explicit null winner when there is no eligible assessment.
- Application binds fixed settings; packaged `cutin search` runs headlessly and
  optionally saves inputs/summary plus best/ using fresh-directory safeguards.
- Verified: 299 tests pass, including fake ranking/bounds/rejection/error/cancel
  cases, consistent artifacts/cleanup, real deterministic target discovery and
  non-target exclusion, outside-repository CLI, and backend-free imports.
  Ruff/build and isolated installed-wheel search/artifact smoke pass.
- Locked seed-17 example: budget five finds two ego collisions; attempt three
  wins. Continue after target discovery to refine within budget. This is empirical
  falsification, not a safety guarantee or a wall-clock-bounded optimizer.

## 6. Web replay — next

- Add a form/API wrapper around existing application workflows.
- Replay completed normalized logs and show metrics in Canvas.
- Do not implement browser physics or backend live stepping in this phase.

## Working practice

Use separate `feat/` branches and small commits. Review shared contracts together.
Run checks with uv; keep runtime dependencies minimal and fully FOSS. Update this
plan with verified completion, not merely the existence of files.

> Recap: The headless/visual single-run MVP now includes independent metrics, application workflows, artifact persistence, and a packaged CLI. Seeded proposals and bounded falsification are implemented; continue with completed-run web replay.
