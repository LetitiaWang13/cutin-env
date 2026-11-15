"""Public import boundaries and the still-unimplemented generation workflow."""

import subprocess
import sys
from pathlib import Path

import pytest

from cutin.config import (
    EvaluationConfig,
    GenerationConfig,
    InterfaceConfig,
    SimulationConfig,
    UserConfig,
)
from cutin.run import run


def test_public_framework_imports_have_no_mvp_or_backend_dependency(tmp_path: Path):
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "import cutin.config, cutin.run, cutin.evaluation.evaluator, sys; "
            "from cutin.simulation.interface import Simulator; "
            "assert not {'cutin_mvp', 'numpy', 'highway_env', 'gymnasium', 'pygame'} "
            ".intersection(sys.modules)",
        ],
        cwd=tmp_path,
        check=True,
    )


def test_workflow_does_not_pretend_to_generate_results():
    config = UserConfig(
        GenerationConfig(),
        SimulationConfig("highway-env", 30, 10),
        EvaluationConfig(),
        InterfaceConfig(),
    )
    with pytest.raises(NotImplementedError):
        run(config)
