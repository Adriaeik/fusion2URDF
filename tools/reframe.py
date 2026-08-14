"""Regenerate frame-dependent robot-description files without Fusion.

Usage from the repository root::

    python tools/reframe.py path/to/my_robot_description

Or, from the directory containing the checkout::

    python -m fusion2URDF.tools.reframe path/to/my_robot_description

The command reads ``debug/frame_model.json`` (normally beside the package),
then applies ``<package>/config/frame_overrides.csv`` and rewrites Xacro,
flat URDF, robot_data.yaml, transform docs, and frame-dependent configs.
Meshes and collision STL files are not touched.
"""

from __future__ import annotations

import argparse
import os
import sys

# Support both ``python tools/reframe.py`` and ``python -m ...tools.reframe``.
_pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_parent = os.path.dirname(_pkg_root)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from fusion2URDF.core.frame_rebaser import (  # noqa: E402
    FRAME_CACHE_FILENAME,
    configure_frames,
    load_frame_cache,
    save_frame_cache,
)
from fusion2URDF.core.data_types import (  # noqa: E402
    CollisionInfo,
    ExportConfig,
)
from fusion2URDF.core.package_generator import (  # noqa: E402
    generate_validation_report,
    regenerate_frame_outputs,
)
from fusion2URDF.core.robot_model import build_model  # noqa: E402
from fusion2URDF.core.snapshot_io import load_snapshot  # noqa: E402
from fusion2URDF.utils.logger import Logger  # noqa: E402


def find_frame_cache(package_dir: str, explicit: str = "") -> str:
    """Find the canonical cache for a generated description package."""
    candidates = []
    if explicit:
        candidates.append(os.path.abspath(explicit))
    candidates.extend((
        os.path.join(package_dir, "debug", FRAME_CACHE_FILENAME),
        os.path.join(os.path.dirname(package_dir), "debug", FRAME_CACHE_FILENAME),
    ))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    tried = "\n  ".join(candidates)
    raise FileNotFoundError(
        f"Could not find {FRAME_CACHE_FILENAME}. Tried:\n  {tried}\n"
        "Run one Fusion export with include_debug=true to create the cache."
    )


def bootstrap_frame_cache(
    package_dir: str,
    snapshot_path: str,
    cache_path: str = "",
    log=None,
) -> str:
    """Create the first canonical cache from an existing legacy export.

    This reuses ``snapshot.json`` plus the meshes already present in the
    package.  It does not call Fusion and does not regenerate mesh files.
    """
    package_dir = os.path.abspath(package_dir)
    snapshot_path = os.path.abspath(snapshot_path)
    if not os.path.isfile(snapshot_path):
        raise FileNotFoundError(f"Snapshot not found: {snapshot_path}")
    log = log or Logger(timestamps=False, quiet=False)
    snapshot = load_snapshot(snapshot_path)
    model = build_model(snapshot, log)

    expected_package = f"{model.name}_description"
    package_name = os.path.basename(os.path.normpath(package_dir))
    if package_name != expected_package:
        raise ValueError(
            f"Snapshot design '{model.name}' expects package "
            f"'{expected_package}', not '{package_name}'"
        )

    config = ExportConfig(
        package_name=package_name,
        output_dir=os.path.dirname(package_dir),
        include_debug=True,
        include_docs=os.path.isdir(os.path.join(package_dir, "docs")),
        include_robot_data_yaml=os.path.isfile(
            os.path.join(package_dir, "robot_data.yaml")
        ),
        include_readme=os.path.isfile(os.path.join(package_dir, "README.md")),
        include_rviz=os.path.isdir(os.path.join(package_dir, "rviz")),
        include_launch=os.path.isdir(os.path.join(package_dir, "launch")),
        include_screenshot=os.path.isfile(
            os.path.join(package_dir, "images", "robot.png")
        ),
        include_ros2_control=False,
    )

    formats = set()
    for link in model.links.values():
        # Mesh paths are deterministic from Phase 2; select the format that is
        # actually on disk in the already-generated package.
        stem = os.path.splitext(link.mesh_visual)[0]
        found_visual = ""
        for extension in (".dae", ".obj"):
            candidate = stem + extension
            if os.path.isfile(os.path.join(package_dir, candidate)):
                found_visual = candidate
                formats.add(extension[1:])
                break
        link.mesh_visual = found_visual or link.mesh_visual
        link.has_visual_mesh = bool(found_visual)

        collision_path = link.mesh_collision
        if collision_path and os.path.isfile(os.path.join(package_dir, collision_path)):
            link.collision = CollisionInfo(
                source="cached_existing",
                mesh_path=collision_path,
            )
        elif found_visual:
            link.collision = CollisionInfo(
                source="visual_reuse",
                mesh_path=found_visual,
            )
        else:
            link.collision = None

    if len(formats) == 1:
        config.visual_format = next(iter(formats))

    if not cache_path:
        cache_path = os.path.join(package_dir, "debug", FRAME_CACHE_FILENAME)
    cache_path = os.path.abspath(cache_path)
    save_frame_cache(model, config, cache_path)
    log(f"  Bootstrapped canonical cache: {cache_path}")
    return cache_path


def reframe_package(
    package_dir: str,
    cache_path: str = "",
    log=None,
    snapshot_path: str = "",
) -> str:
    """Apply CSV overrides and rewrite descriptions; return cache path used."""
    package_dir = os.path.abspath(package_dir)
    if not os.path.isdir(package_dir):
        raise FileNotFoundError(f"Package directory not found: {package_dir}")

    if snapshot_path:
        # Supplying a snapshot is an explicit request to (re)build the cache;
        # useful for packages exported before frame_model.json existed.
        cache_path = bootstrap_frame_cache(
            package_dir,
            snapshot_path,
            cache_path=cache_path,
            log=log,
        )
    else:
        cache_path = find_frame_cache(package_dir, cache_path)
    model, config = load_frame_cache(cache_path)
    config.output_dir = os.path.dirname(package_dir)
    log = log or Logger(timestamps=False, quiet=False)

    log.section("OFFLINE FRAME REGENERATION")
    log(f"  Package: {package_dir}")
    log(f"  Cache:   {cache_path}")
    configure_frames(model, config, package_dir, log)
    regenerate_frame_outputs(model, config, package_dir, log)

    debug_dir = os.path.dirname(cache_path)
    if os.path.isdir(debug_dir):
        generate_validation_report(model, config, debug_dir)
        log(f"  -> {os.path.join(debug_dir, 'validation.md')}")

    log.section("OFFLINE FRAME REGENERATION COMPLETE")
    return cache_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Reapply frame_overrides.csv to a cached Fusion export without "
            "extracting or exporting meshes again."
        )
    )
    parser.add_argument("package", help="Generated *_description package directory")
    parser.add_argument(
        "--cache",
        default="",
        help="Explicit debug/frame_model.json path (normally auto-detected)",
    )
    parser.add_argument(
        "--snapshot",
        default="",
        help=(
            "Bootstrap/rebuild the cache from an existing debug/snapshot.json "
            "and package meshes; no Fusion API is used"
        ),
    )
    args = parser.parse_args(argv)
    try:
        reframe_package(args.package, args.cache, snapshot_path=args.snapshot)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
