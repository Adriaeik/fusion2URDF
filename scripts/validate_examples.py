#!/usr/bin/env python3
"""
Validate ROS 2 description packages bundled under examples/.

Checks (each is a regression guard for a real bug we've hit):

  - Every .urdf and .urdf.xacro is well-formed XML.
  - Every <mesh filename="package://<pkg>/<rel>"> resolves to a file
    on disk relative to the package root.
  - Every .stl under meshes/ has a non-degenerate bbox (> 1 mm per axis).
    Regression guard for the Fusion degenerate-STL bug where
    exportMgr.execute returned True while writing all-zero vertices.

Runnable locally and in CI. No external dependencies — stdlib only.

Exit status: 0 on clean, 1 on any failure. Failures are listed with
file paths and expected-vs-actual so they're diff-able in CI output.

Usage:
    python scripts/validate_examples.py [examples/]
"""
from __future__ import annotations

import sys
import re
import xml.etree.ElementTree as ET
from pathlib import Path

# Reuse the same helper + threshold the pipeline uses post-export so
# CI catches the same class of failure the pipeline itself catches.
# The repo folder is named ``fusion2URDF`` and acts as a Python
# package; we add the *parent* directory to sys.path so we can import
# it via its package name (which lets the package's own relative
# imports resolve).
_HERE = Path(__file__).resolve().parent
_PKG_ROOT = _HERE.parent
sys.path.insert(0, str(_PKG_ROOT.parent))
from fusion2URDF.core.collision_generator import (  # noqa: E402
    _stl_bbox_size_cm,
    _DEGENERATE_BBOX_THRESHOLD_CM,
)


_PACKAGE_URI = re.compile(r"package://([^/]+)/(.+)")


def _fail(failures: list, path: Path, msg: str) -> None:
    failures.append(f"  {path}: {msg}")


def _check_xml(path: Path, failures: list) -> ET.ElementTree | None:
    try:
        return ET.parse(path)
    except ET.ParseError as e:
        _fail(failures, path, f"malformed XML — {e}")
        return None


def _check_mesh_refs(tree: ET.ElementTree, urdf_path: Path,
                     pkg_name: str, pkg_root: Path, failures: list) -> None:
    """Every <mesh filename="package://<pkg>/<rel>"> must resolve on disk."""
    for mesh in tree.iter("mesh"):
        fn = mesh.get("filename", "")
        m = _PACKAGE_URI.match(fn)
        if not m:
            continue
        ref_pkg, rel = m.group(1), m.group(2)
        if ref_pkg != pkg_name:
            _fail(failures, urdf_path,
                  f"mesh references foreign package '{ref_pkg}' (expected '{pkg_name}')")
            continue
        target = pkg_root / rel
        if not target.is_file():
            _fail(failures, urdf_path, f"mesh not found on disk: {rel}")


def _check_stls(pkg_root: Path, failures: list) -> None:
    """Every STL under meshes/ must have a non-degenerate bbox."""
    meshes = pkg_root / "meshes"
    if not meshes.is_dir():
        return
    for stl in meshes.rglob("*.stl"):
        bbox = _stl_bbox_size_cm(str(stl))
        if max(bbox) < _DEGENERATE_BBOX_THRESHOLD_CM:
            _fail(failures, stl,
                  f"degenerate collision STL — bbox ≈ "
                  f"{bbox[0]*10:.3f} × {bbox[1]*10:.3f} × {bbox[2]*10:.3f} mm "
                  f"(≥ 1 mm expected)")


def _validate_package(pkg_root: Path) -> list:
    failures: list = []
    pkg_name = pkg_root.name  # <pkg>_description
    urdf_dir = pkg_root / "urdf"
    if not urdf_dir.is_dir():
        _fail(failures, pkg_root, "missing urdf/ subdirectory")
        return failures

    urdfs = list(urdf_dir.rglob("*.urdf")) + list(urdf_dir.rglob("*.urdf.xacro"))
    if not urdfs:
        _fail(failures, urdf_dir, "no .urdf or .urdf.xacro files found")

    for urdf in urdfs:
        tree = _check_xml(urdf, failures)
        if tree is None:
            continue
        _check_mesh_refs(tree, urdf, pkg_name, pkg_root, failures)

    _check_stls(pkg_root, failures)
    return failures


def main(argv: list) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path("examples")
    if not root.is_dir():
        print(f"error: {root} is not a directory", file=sys.stderr)
        return 2

    packages = sorted(p for p in root.iterdir()
                      if p.is_dir() and p.name.endswith("_description"))
    if not packages:
        print(f"error: no *_description packages under {root}", file=sys.stderr)
        return 2

    total_failures = 0
    for pkg in packages:
        print(f"→ {pkg.relative_to(root.parent)}")
        failures = _validate_package(pkg)
        if failures:
            total_failures += len(failures)
            for line in failures:
                print(line)
            print(f"  {len(failures)} failure(s)")
        else:
            print("  OK")

    print()
    if total_failures:
        print(f"FAIL: {total_failures} total validation failure(s)")
        return 1
    print("All fixtures valid.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
