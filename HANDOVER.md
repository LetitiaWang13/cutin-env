# Developer handover

## Status and starting point

The local MVP is implemented: configuration → controlled highway-env simulation
→ independent safety metrics → explicit artifacts; seeded sampling → bounded
falsification; Web scenario input → completed-log replay. Recent refinements add
world-anchored lane motion, grouped/sliding inputs, actionable validation, replay-
ready feedback, and timestamped frontend run IDs.

Use `cutin/` as the working directory. `cutin-v0/` is reference material, never a
runtime dependency. Changes were made on chained feat/ branches; verify the
checkout contains the latest Web feedback changes rather than assuming main
has been merged. `AGENTS.md` was intentionally left untracked/unmodified.

Read README.md for all runnable commands, DESIGN.md for decisions, and PLAN.md
for verified history. Code/comments/docs should remain English; use uv and FOSS
libraries without telemetry, with small feature branches and commits.

## Component ownership

| Location | Owns |
| --- | --- |
| core/ | Scenario, state/action/log/metric records; shared rectangle projection |
| simulation/config.py | Numerical rates and actuator bounds |
| simulation/controllers.py | Fresh per-run following, lane keeping, timed merge |
| simulation/runner.py | Fixed clock, policy scheduling, termination, partial logs |
| simulation/adapters/ | Scenario/action/state translation; CutInEnv joint stepping and optional native renderer |
| evaluation/ | Offline collision/gap/TTC formulas, validity and warning thresholds |
| generation/ | Allowed ranges and single-draw local-RNG proposals |
| falsification/ | Eligibility, lexicographic ranking, budget/cap and counters |
| application/ | Thin composition and explicit loading/saving |
| interfaces/cli.py | Command parsing, native viewer, terminal summaries |
| interfaces/web/ | Local HTTP boundary and self-hosted recorded-pose UI |

Do not move formulas into CLI/Web, add physics to Canvas, or create a second
search loop in application. Core/generation/evaluation/falsification imports
must not load backend objects. Tests assert these import boundaries.

## Contracts and invariants to preserve

- ScenarioConfig is concrete; SearchSpace contains ranges, with a separate valid
  base. Scene v1 has three lanes: ego/front initially lane 1, cut-in lane 0.
  StateSnapshot itself has no fixed count or unique-role assumption. Its plural
  role lookup returns all matches; role-pair metrics aggregate all combinations.
- Boundary validation, not defensive dataclass __post_init__ hooks. Parsing or
  execution validates inputs; internal frozen records remain simple. Metadata
  dictionaries are not a general immutable-container framework.
- StateSnapshot.tick is the physics clock. Seconds = tick / physics_hz. Logs
  include tick 0 and every physical tick. Controllers run at controller_hz and
  hold commands between updates. CutInEnv steps the road once per call; avoid
  inherited Gym multi-frame scheduling, random traffic, or keyboard controls.
- Gym rewards are zero and unrelated to safety. No RL training is required.
  Backend supports 5 m × 2 m vehicles and speed <=40 m/s; unsupported model
  values are explicit failures, not silently adjusted parameters.
- Backend collision flags establish collision, not negative projected gap.
  TTC is positive gap / positive x-projected closing speed only when lateral
  bounds overlap. It cannot predict an approaching sideways cut-in. Null is not
  zero or safe; minima and their first exact-tie ticks remain independent.
- Complete assessment means a horizon or target collision, not safe. Errors,
  road exits, cancellation, missing ego observations, and non-target collisions
  never become ranking successes. Only ego-involving collisions are targets.
- Search budget counts eligible assessments; attempt cap counts every proposal.
  One outcome bucket per attempt; ego_collisions is a subset of completed.
  Rank collision first, then smaller ego-pair gap, then positive TTC; missing
  values last, ties preserve earlier attempts. Warnings do not rank candidates.
- Sampling uses the caller's local Random and sorted path order. Fixed bounds
  consume no draw; scenario seed remains unchanged. RejectedSample counts one
  attempt, never hides an internal retry. Configuration errors fail upfront.
- Each run has fresh policies/backend scene. Search keeps only the best full
  trace, counters, and last diagnostics, not a growing archive of all traces.
- Output locations are caller choices; never overwrite existing artifacts.
  Serialization precedes directory creation; write failures clean the new
  directory. This is not a database, crash-atomic store, or resumable job system.

## UI and Web specifics

The standard-library HTTP server binds only 127.0.0.1. Preserve Host/Origin,
mutation-header, body/resource and path-root checks. Limits: JSON 64 KiB, Web
horizon 60 s, physics rate 120 Hz, each replay file 16 MiB. One run at a time;
parallel mutations get 409. This is local trusted development, not public hosting.

Web-only initial clearance adds 0.1 m to vehicle length when lateral footprints
intersect. It is mirrored in validation.py and initialPositionIssues in app.js,
with 1e-9 m numerical comparison tolerance. Keep client/server checks aligned.
It does not change core/CLI/search admissibility; adjacent-lane same-x setups
remain allowed when lateral footprints are disjoint. No automatic correction.

Numbers are authoritative over convenience slider ranges/steps. Preserve exact
payloads and preset synchronization. Form edits describe the next scenario;
the right panel is a completed recorded run. On success it is already loaded;
Play begins replay, and a navigation button focuses it. Empty/error runs must
not receive a misleading replay-ready message.

Canvas selects recorded ticks without integrating/interpolating dynamics. Camera
follows ego; dash phase is world-x anchored (including clipped road starts), not
time animated. Pause and seek must freeze/restore exact phase. Do not regress
into stationary screen-anchored dashes or animate a stationary ego artificially.

New names: frontend-YYYYMMDD-HHMMSSZ-xxxxxxxx, UTC plus random UUID suffix. Old
hash-only IDs and CLI names remain readable. IDs are safe single path components;
do not introduce slash-containing source prefixes or arbitrary filesystem routes.

## Artifacts, replay, and reproducibility

Single run: scenario.json, simulator.json, evaluation.json, log.json, metrics.json.
Evaluation is configuration; Metrics is measured results with resolved thresholds.
Search: base.json, space.json, simulator.json, evaluation.json, search.json,
optionally best/ containing the five winning run files. Null best_config/metrics/
best_run and absent best/ are intentional when no candidate is eligible.

Web reads raw persisted JSON through application.storage without re-evaluating.
Selecting a search directory loads its best/ automatically. Legacy script output
is a single raw log, not the five-file replay bundle. Typed log/metric decoding
and arbitrary uploads remain deferred.

`cutin run SAVED/scenario.json --render` re-simulates, using default numerical and
warning settings. It is not original replay and does not automatically restore
customized simulator/evaluation files. Use Web for exact recorded playback.
For explicit in-memory restoration (trusted saved configuration):

```python
import json
from pathlib import Path
from cutin.application import load_scenario, run_scenario
from cutin.evaluation import EvaluationConfig
from cutin.simulation import SimulatorConfig

root = Path("runs/YOUR-RUN-ID")
scenario = load_scenario(root / "scenario.json")
simulator = SimulatorConfig(**json.loads((root / "simulator.json").read_text()))
evaluation = EvaluationConfig(**json.loads((root / "evaluation.json").read_text()))
result = run_scenario(scenario, simulator, evaluation)
```

To view that re-simulation natively, call interfaces.cli.run_with_viewer with
those same three arguments. It is a local interface helper, not a live Web API.
Reproducibility also depends on backend/dependency versions recorded in metadata
and uv.lock; do not promise identical trajectories across library upgrades.

## Verification and useful regressions

- `uv run --locked pytest -q`, Ruff lint/format checks, and `uv build`.
- Optional `scripts/smoke_web_browser.py` with uv --with playwright; install
  Chromium first. Node tests skip if Node is absent; browser smoke covers linked
  controls, exact payloads, feedback, motion pixels, seeking, and mobile layout.
- Baseline: 10 s/300 ticks at 30 Hz, no collision; cut-in start tick 60,
  observed completion tick 155 in the locked setup.
- Unsafe: ego/cut-in collision tick 18 (0.6 s). Its positive longitudinal TTC is
  null because lateral intrusion enters already-overlapping x footprints.
- Seed-17 example search budget 5: two eligible ego collisions, attempt 3 wins.
  Real non-target collision regressions verify no winner/unsafe-success leakage.
- Keep rendered/headless and pre-Gym/primitive trace equivalence tests. Native
  desktop appearance/input is distinct from browser/SDL-dummy verification.

## Recommended next development

1. Improve scenario admissibility/search quality. The current broad absolute-x
   ranges can find trivial near-coincident, early side impacts. Define explicit
   physical/preparation requirements and reaction opportunity before optimizing;
   report invalid/rejected attempts honestly. Retain unsafe.toml as a diagnostic,
   not a model of a meaningful discovered scenario. Do not globally ban adjacent
   same-x initialization merely to hide trivial cases.
2. Separate proposal/admissibility/objective concerns. Relative position at merge
   start differs from initial absolute x. PREPARE is deferred, not discarded.
   Add its semantics and synthetic cases before changing search dimensions.
3. Calibrate following/tracking gains and compare useful near-miss/collision cases.
   TTC alone misses lateral threats; changing metric semantics requires a version
   bump and explicit regression evidence, not a new arbitrary composite score.
4. Consider alternative search proposals behind unchanged callable contracts,
   with frozen settings, reproducible seeds, consistent counters/artifacts.
5. Only after a real need: job-based Web search, typed reload/re-evaluation,
   progress/cancel/resume, persistence indexes, or remote/multi-user deployment.
   These require new security/lifecycle design, not removing loopback guards.

Do not add registries, plugin frameworks, hosted telemetry, full OpenSCENARIO,
RL rewards, or real-time sessions solely as placeholders. Preserve the small,
inspectable workflow first. Empirical falsification is never a proof of safety.
