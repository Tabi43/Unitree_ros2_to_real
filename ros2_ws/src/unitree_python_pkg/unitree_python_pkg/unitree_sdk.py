#!/usr/bin/env python3
"""Helpers to load Unitree legged SDK Python wrapper (robot_interface)."""

import importlib
import os
import platform
import sys
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


def _arch_name() -> str:
    machine = platform.machine().lower()
    if machine in ("x86_64", "amd64"):
        return "amd64"
    if machine in ("aarch64", "arm64"):
        return "arm64"
    raise RuntimeError(f"Unsupported architecture for Unitree SDK: {machine}")


def _candidate_sdk_paths() -> list:
    arch = _arch_name()
    candidates = []

    env_path = os.environ.get("UNITREE_LEGGED_SDK_PYTHON_PATH")
    if env_path:
        candidates.append(Path(env_path))

    try:
        prefix = Path(get_package_prefix("unitree_legged_sdk"))
        candidates.append(prefix / "lib" / "python" / arch)

        # Typical colcon workspace layout:
        # <ws>/install/unitree_legged_sdk -> <ws>/src/unitree_legged_sdk
        workspace_root = prefix.parent.parent
        candidates.append(
            workspace_root / "src" / "unitree_legged_sdk" / "lib" / "python" / arch
        )
    except PackageNotFoundError:
        pass

    # Extra fallback: walk ancestors and look for a sibling src/unitree_legged_sdk.
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        candidates.append(parent / "src" / "unitree_legged_sdk" / "lib" / "python" / arch)

    unique = []
    seen = set()
    for item in candidates:
        item_str = str(item)
        if item_str in seen:
            continue
        seen.add(item_str)
        unique.append(item)
    return unique


def load_robot_interface():
    """Import and return the SDK wrapper module named `robot_interface`."""
    try:
        return importlib.import_module("robot_interface")
    except ImportError as first_error:
        checked_paths = []
        for path in _candidate_sdk_paths():
            if not path.is_dir():
                continue

            checked_paths.append(str(path))
            path_str = str(path)
            if path_str not in sys.path:
                sys.path.insert(0, path_str)

            try:
                return importlib.import_module("robot_interface")
            except ImportError:
                continue

        checked = "\n  - ".join(checked_paths) if checked_paths else "(no valid paths found)"
        raise ImportError(
            "Unable to import Unitree SDK Python wrapper `robot_interface`.\n"
            "Set UNITREE_LEGGED_SDK_PYTHON_PATH to the SDK python lib directory if needed.\n"
            f"Checked paths:\n  - {checked}"
        ) from first_error