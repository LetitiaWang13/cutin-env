"""Numeric values remain authoritative over bounded, stepped quick-adjust sliders."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_slider_precision_and_range_hints_do_not_transform_numeric_values():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is optional; Chromium smoke exercises linked controls")
    asset = (
        Path(__file__).resolve().parents[2] / "src/cutin/interfaces/web/assets/app.js"
    )
    code = """
import fs from 'node:fs'; import assert from 'node:assert/strict';
const text = fs.readFileSync(process.argv[1], 'utf8');
const source = 'data:text/javascript;base64,' +
  Buffer.from(text).toString('base64');
const {sliderHint} = await import(source);
assert.equal(sliderHint(25, 25, 0, 40, 0.1), '');
assert.equal(sliderHint(0, 0, 0, 40, 0.1), '');
assert.equal(sliderHint(NaN, 25, 0, 40, 0.1), '');
assert.match(sliderHint(26.425, 26.4, 0, 40, 0.1), /Exact number retained/);
assert.match(sliderHint(1234.125, 200, 2.5, 200, 0.5), /2.5–200/);
assert.match(sliderHint(0.01, 0.1, 0.1, 10, 0.1), /step 0.1/);
"""
    subprocess.run([node, "--input-type=module", "-e", code, str(asset)], check=True)
