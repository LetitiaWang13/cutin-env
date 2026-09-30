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
  partial logs, and cleanup. No driving policies are implied by simulate() yet.
- Verified: 103 tests pass, including real backend stepping and fake-backed
  execution, plus Ruff and package build/import checks.

## 3b. Controlled simulation — next

- Add car-following, lane keeping, and timed cut-in (PREPARE remains deferred).
- Extend the existing runner with same-snapshot decisions at controller ticks
  and held actions between those ticks; keep backend translation unchanged.
- Check repeatability and that scenario parameter changes affect actual motion.

Acceptance: ScenarioConfig -> SimulationLog on the real backend.

## 4. Evaluation and user entry point

- Define Metrics and implement collision/gap/TTC using synthetic logs first.
- Connect the evaluator to real simulation logs.
- Add thin application workflows, explicit artifact storage, and a CLI.
- Establish benign/unsafe regression examples and complete the headless MVP.

## 5. Generation and falsification

- Add seeded uniform sampling, then bounded sequential random search.
- Test search with injected fake simulator/evaluator callables before integration.
- Save the winning scenario and summary. Non-ego collisions are not successes;
  truncated non-target collision runs are excluded from ranking.

## 6. Web replay

- Add a form/API wrapper around existing application workflows.
- Replay completed normalized logs and show metrics in Canvas.
- Do not implement browser physics or backend live stepping in this phase.

## Working practice

Use separate `feat/` branches and small commits. Review shared contracts together.
Run checks with uv; keep runtime dependencies minimal and fully FOSS. Update this
plan with verified completion, not merely the existence of files.

> Recap: Build and verify contracts first, then backend stepping, controlled simulation, evaluation/CLI, search, and finally completed-run web replay.
