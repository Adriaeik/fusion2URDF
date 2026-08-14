#!/usr/bin/env python3
"""
Run tests from within the fusion2URDF directory.

Usage:
    cd fusion2URDF
    python run_tests.py
"""
import subprocess
import sys
import os

# Run from parent directory so package imports work
parent = os.path.dirname(os.path.abspath(__file__))
grandparent = os.path.dirname(parent)

result = subprocess.run(
    [sys.executable, '-m', 'fusion2URDF.tests.test_core'],
    cwd=grandparent
)

if result.returncode != 0:
    sys.exit(result.returncode)

result = subprocess.run(
    [sys.executable, '-m', 'fusion2URDF.tests.test_robot_model'],
    cwd=grandparent
)

if result.returncode != 0:
    sys.exit(result.returncode)

result = subprocess.run(
    [sys.executable, '-m', 'fusion2URDF.tests.test_frame_rebaser'],
    cwd=grandparent
)

if result.returncode != 0:
    sys.exit(result.returncode)

result = subprocess.run(
    [sys.executable, '-m', 'fusion2URDF.tests.test_export_pipeline'],
    cwd=grandparent
)
sys.exit(result.returncode)
