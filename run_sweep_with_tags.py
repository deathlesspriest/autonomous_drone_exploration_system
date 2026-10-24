#!/usr/bin/env python3
"""Updated launcher: one integrated mission with conversion and recording.

Run from the course repository root. This replaces the previous two-process
launcher. All arguments are passed to laptop/mini_project_mission.py.
"""
# PROJECT ADDITION: this whole launcher is absent from the original GitHub repo.
# UPDATED FROM PREVIOUS STARTER: run one integrated program instead of two subprocesses.
from pathlib import Path
import sys

# PROJECT ADDITION: make laptop/ imports available and pass command-line options through.
if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parent / 'laptop'))
    from mini_project_mission import main
    raise SystemExit(main())
