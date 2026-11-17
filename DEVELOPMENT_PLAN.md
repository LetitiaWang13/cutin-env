# Component Development Plan

This plan groups the remaining work by functionality, not by people. Components
can progress in parallel once their shared input/output meanings are agreed.
Keep the implementation small; do not introduce another framework or duplicate
existing workflows.

## 1. Existing foundation

The following path already works:

```text
ScenarioConfig -> HighwayEnvAdapter.run() -> ScenarioRecord -> JSON
```

- Concrete inputs: [scenario.py](src/cutin/core/scenario.py).
- Execution settings: [config.py](src/cutin/config.py).
- Recorded motion: [record.py](src/cutin/core/record.py).
- Real native execution: [adapter.py](src/cutin/simulation/adapters/highway_env/adapter.py).
- Runnable example: [run_simulation.py](examples/run_simulation.py).

Ego/front use native IDM without MOBIL. Cut-in uses WARMUP -> PREPARE -> MERGE ->
FOLLOW: an initial-speed target before completion, then native IDM. The record
contains actual states/actions, phase timing, and execution versions.

The MVP remains a reference under `src/cutin_mvp/`, `tests/mvp/`, `examples/mvp/`,
and `docs/mvp/`. New functionality should not import its runtime as a shortcut.

## 2. Input, output, and CLI

**Main files:** `src/cutin/interfaces/cli.py`, `src/cutin/run.py`, `src/cutin/config.py`.

Work:

- Load agreed user/scenario/execution settings from configuration files.
- Expose direct simulation first, then generation when its workflow is ready.
- Save and load records using the agreed JSON structure.
- Share loading and workflow code with Web rather than maintaining two versions.
- Keep saving/playback outside the simulator and formulas outside the interface.

**First deliverable:** configuration -> real simulation -> saved record, without
requiring the optimizer. Typed loading and configuration layout should be agreed
before creating separate parsers in CLI and Web.

## 3. Recorded playback and Web

**Main area:** `src/cutin/interfaces/`; reference assets are in
`src/cutin_mvp/interfaces/web/assets/`.

Work:

- Adapt the MVP player to the new `ScenarioRecord.frames` structure.
- Render road geometry and recorded vehicle positions, headings, and dimensions.
- Support play/pause, seeking, speed selection, time, and cut-in phase display.
- Add metric display once evaluation output is agreed.
- Connect generation controls after the shared workflow is available.

**First deliverable:** load JSON produced by `examples/run_simulation.py` and replay
it. Playback selects recorded frames; it must not integrate physics or silently
re-simulate the configuration. A new Web job/progress system is not required for
this milestone.

## 4. Metrics and evaluator

**Main files:** `src/cutin/evaluation/metrics/`, `src/cutin/evaluation/evaluator.py`.

Work:

- Agree on TTC variant, vehicle pairs, evaluation windows, and unavailable values.
- Compute actual LCT from recorded merge start/completion times; unfinished
  maneuvers remain unavailable, not zero.
- Implement the selected robustness/specification semantics after those meanings
  are settled. Horizon completion alone does not establish safety.
- Keep metric and robustness computation independent of generator/backend objects.
- Return evaluation separately from the motion record, with an unambiguous
  association to its scenario/record.

**First deliverable:** record -> documented TTC/LCT and collision information,
verified on hand-constructed traces and real simulation records. Then add
robustness and target-compliance results. Do not invent missing formula semantics.

## 5. Generation translators and candidate construction

**Main area:** `src/cutin/generation/optim/translators/`.

Work:

- Aggressiveness translator: level/profile -> resolved metric/specification
  parameters. This belongs to generation, not the evaluator.
- Search-space translator: fixed user conditions and targets -> tunable dimensions
  and ranges, considering warmup and constant-speed reachability.
- Map proposed values into a complete `ScenarioConfig`, preserving fixed ego/front
  conditions and controller assumptions. This can be a simple conversion function.

The first search variables are initial relative position, initial speed, target
merge-start gap, and lane-change reference duration. Relative positions or
percentages resolve into world x before simulation. FOLLOW settings, tracking
gains, and warmup are fixed unless an experiment explicitly changes the scope.

**First deliverable:** an explicit profile and search space that can produce
concrete runnable scenarios. Coordinate specification parameters with metrics and
parameter units/meaning with simulation; do not duplicate robustness formulas.

## 6. Optimization methods

**Main area:** `src/cutin/generation/optim/optimizers/`.

Work:

- Implement random candidate proposal through the optimizer interface first.
- Add BO/GP proposal and observation updates once the objective is settled.
- Keep simulation/evaluator execution out of individual optimization methods.
- Do not turn unavailable scores or execution errors into fabricated GP targets.
- Compare random search and BO using the same parameterization, evaluation,
  stopping/accounting rules, and reported simulation cost.

**First deliverable:** random candidates that the generator can execute. BO's
initial samples, acquisition choices, and tuning are a later step, not blockers
for the first complete generation workflow.

## 7. Generator integration and shared data

**Main files:** `src/cutin/generation/optim/optim_generator.py`, `src/cutin/run.py`,
`src/cutin/config.py`, `src/cutin/core/`.

This is a distinct work block: implementing translators and optimizer methods
alone does not produce a complete generator.

`OptimGenerator.run()` owns the single optimization loop:

```text
Resolve targets and search space
    -> Propose candidate
    -> Construct ScenarioConfig
    -> Simulate
    -> Evaluate
    -> Feed back observations
    -> Repeat / select results / return
```

Agree on budget/stop semantics, requested result count, and selection before
implementing them. Primary outputs are target-compliant safe-boundary scenarios;
failure cases can be returned separately. Return fewer results when the search
has not found enough, rather than implying success.

The outer `run.py` connects the user workflow, not another search loop. Integration
must preserve fresh episodes, correct clocks, phase timing, and record semantics;
backend fixes and their regression tests remain in the simulation work area.

Shared config/model edits should be coordinated rather than independently changing
field meanings in different branches. Extend the existing small models only when
needed; avoid generic schemas, registries, and speculative abstractions.

## 8. Agreements needed before parallel integration

- TTC/LCT definitions, time windows, robustness sign/formula, and target compliance.
- Search parameter names, units, range meaning, and candidate-to-scenario mapping.
- Evaluation output structure and its association with returned scenarios/records.
- UserConfig sections, loading/saving format, and which values are fixed/searched.
- Common budget, stopping, and result-selection semantics for random search and BO.

Independent work can begin before every integration detail is coded: playback has
real JSON input, metrics can use synthetic traces, and optimization methods can
use a simple numerical objective during their own tests. Test doubles must not be
presented as real simulation or evaluation results.

## 9. Delivery milestones

1. **Done:** direct scenario execution and JSON-compatible records.
2. **Parallel next:** saved-record playback, basic metrics/evaluator, explicit
   translators, random proposal, and input/output handling.
3. **First integrated loop:** random generation -> simulation -> evaluation ->
   selected configurations/records/scores -> save -> replay.
4. **Then:** BO/GP using the same execution path, with a random-search comparison.

Work on `feat/` branches with small commits. Use `uv run --locked pytest -q`, Ruff
lint/format checks, and `uv build` for integration checks. Keep the MVP separate.

> Recap: Build on the working scenario-to-record pipeline. Develop interfaces, playback, metrics, translators, and optimizer methods in parallel, with explicit ownership of the generator integration loop and coordinated shared data semantics.
