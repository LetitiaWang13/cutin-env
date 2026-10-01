"""Check installed imports without relying on the repository working directory."""

import subprocess
import sys
from importlib.metadata import distribution
from pathlib import Path


def test_installed_distribution_includes_mit_license() -> None:
    package = distribution("cutin")
    assert package.metadata["License-Expression"] == "MIT"
    assert package.metadata.get_all("License-File") == ["LICENSE"]
    text = package.read_text("licenses/LICENSE")
    assert text is not None
    assert "MIT License" in text
    assert "Cut-in contributors" in text
    assert 'THE SOFTWARE IS PROVIDED "AS IS"' in text


def test_installed_import_has_no_backend_dependencies(tmp_path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "from cutin.core import ScenarioConfig; "
            "from cutin.simulation import SimulatorConfig, simulate; "
            "import sys; "
            "assert ScenarioConfig().road.lane_count == 3; "
            "assert not {'highway_env', 'pygame', 'numpy', 'gymnasium'} "
            ".intersection(sys.modules)",
        ],
        cwd=tmp_path,
        check=True,
    )
