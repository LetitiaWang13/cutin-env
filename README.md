# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for the delivery sequence.

## Current scope

Step 1 implements validated scenario configuration and immutable state/log
contracts only. There is no simulation or CLI yet, and no runtime dependencies.
All numerical defaults are illustrative, not safety-calibrated.

```sh
uv sync --locked
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv build
```

Load the example through the installed package:

```python
import json
from pathlib import Path

from cutin.core import ScenarioConfig

scenario = ScenarioConfig.from_dict(
    json.loads(Path("examples/baseline.json").read_text())
)
print(scenario.cut_in.start_time_s)
```

Core accepts plain JSON values; callers own file I/O. Unknown fields and invalid
values fail explicitly. Input defaults are expanded by `to_dict()`. Cut-in
`start_time_s` means the start of the lateral reference (at the next control
tick), not the start of position preparation. PREPARE is deferred.

Road coordinates use SI units: x runs along the road, lane centres are
`lane * lane_width_m`, and heading is radians from positive x. Vehicle positions
refer to centres. The baseline uses ego/front in lane 1 and cut-in in lane 0;
lane 2 is empty.

State/log records are in-memory contracts for the upcoming integration spike.
Backend settings, provenance details, log serialization, and applied-action
bounds will be finalized in that step, not guessed by the foundation.
