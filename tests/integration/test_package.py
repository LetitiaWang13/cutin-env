"""Check installed imports without relying on the repository working directory."""

import subprocess
import sys
from pathlib import Path


def test_installed_import_has_no_backend_dependencies(tmp_path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "from cutin.core import ScenarioConfig; "
            "import sys; "
            "assert ScenarioConfig().road.lane_count == 3; "
            "assert not {'highway_env', 'pygame', 'numpy', 'gymnasium'} "
            ".intersection(sys.modules)",
        ],
        cwd=tmp_path,
        check=True,
    )
