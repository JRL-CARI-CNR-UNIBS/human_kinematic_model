"""pytest configuration for the python tests.

* enables JAX 64-bit floats, needed to compare the JAX model against the double-precision C++ one
  (float32 inputs keep working, the JAX model preserves the input dtype);
* puts ``scripts/`` (python translation and JAX model) on ``sys.path``;
* puts the directory of the ``human_model_binding`` extension on ``sys.path``: ``$HUMAN_MODEL_BINDING_DIR``
  if set, otherwise ``build/python`` (see the build command in CLAUDE.md). A binding already on
  ``PYTHONPATH`` (e.g. from a colcon install) is used if neither exists.
"""
import os
import sys
from pathlib import Path

import jax

jax.config.update("jax_enable_x64", True)

REPO_ROOT = Path(__file__).resolve().parents[2]

sys.path.insert(0, str(REPO_ROOT / "scripts"))

_binding_dir = Path(os.environ.get("HUMAN_MODEL_BINDING_DIR", REPO_ROOT / "build" / "python"))
if _binding_dir.is_dir():
    sys.path.insert(0, str(_binding_dir))
