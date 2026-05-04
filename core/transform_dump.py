"""
Transform dump — debug helper that records Fusion's authoritative
occurrence transforms next to the exporter's chain-walked values.

Called from the main plugin during every export to drop a
``debug/fusion_transforms.json`` alongside the snapshot.  Useful when
diagnosing pose/orientation mismatches: compare the JSON's
``world_position_xyz_cm`` (composed via 4×4 assemblyContext chain) to
``snapshot.global_position`` (naive translation sum, no parent
rotations applied).

REQUIRES FUSION 360 API — call from inside the plugin.

Author: Adrian Valaker Eikeland
"""

import adsk.core
import adsk.fusion
import json


def _matrix3d_to_list(m):
    """Convert a Fusion Matrix3D to a 16-element row-major list."""
    if m is None:
        return None
    return [float(v) for v in m.asArray()]


def _matrix3d_translation(m):
    """Translation in cm from a row-major 4×4 Matrix3D."""
    if m is None:
        return [0.0, 0.0, 0.0]
    a = m.asArray()
    return [float(a[3]), float(a[7]), float(a[11])]


def _matrix3d_rotation_3x3(m):
    """3×3 rotation as row-major 9-tuple from a 4×4 Matrix3D."""
    if m is None:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    a = m.asArray()
    return [float(a[0]), float(a[1]), float(a[2]),
            float(a[4]), float(a[5]), float(a[6]),
            float(a[8]), float(a[9]), float(a[10])]


def _world_pose_from_transform2(occ):
    """Read the occurrence's world pose directly from ``transform2``.

    Fusion's ``transform2`` is *already* composed through the
    assemblyContext chain on Fusion's side — despite the API doc string
    saying "relative to parent component."  Empirically, for an
    occurrence at depth 1 like ``pendel_with_esp+pendel`` (where pendel
    sits at local (0,0,0) within pwe), ``pendel.transform2.translation``
    equals ``pendel_with_esp.transform2.translation`` — i.e. it's
    already pwe's world pose with pendel's identity local transform
    composed.

    The previous version of this function walked the assemblyContext
    chain and multiplied transform2 at every level, which doubly
    composed the parent and produced wrong values.  The single read
    below is the source of truth Fusion intends.
    """
    if occ is None:
        return [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    try:
        t2 = occ.transform2
    except Exception:
        return [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return _matrix3d_translation(t2), _matrix3d_rotation_3x3(t2)


def dump_transforms(design, out_path: str) -> int:
    """Walk every occurrence and dump local + world transforms to JSON.

    Returns the number of occurrences written.  Raises on file I/O or
    Fusion API errors so the caller can log the failure cleanly.
    """
    root = design.rootComponent
    out = {
        "design_name": root.name,
        "_units": "translations in cm; rotation 3x3 row-major",
        "_note": ("transform = old 'relative to parent' (often buggy); "
                  "transform2 = world pose (despite API doc claiming "
                  "'relative to parent', empirically the chain is "
                  "already composed); world_position copied from "
                  "transform2.translation as the canonical truth"),
        "occurrences": {},
    }

    for occ in root.allOccurrences:
        if not occ or not occ.component:
            continue
        try:
            full_path = occ.fullPathName or ""
            comp_name = occ.component.name
            clean = comp_name.rsplit(' v', 1)[0] if ' v' in comp_name else comp_name

            t_local = _matrix3d_to_list(occ.transform)
            t2_local = _matrix3d_to_list(occ.transform2)
            world_xyz, world_rot = _world_pose_from_transform2(occ)

            try:
                is_subasm = occ.childOccurrences and occ.childOccurrences.count > 0
            except Exception:
                is_subasm = False

            depth = max(0, len(full_path.split('+')) - 1)

            out["occurrences"][full_path] = {
                "clean_name": clean,
                "is_subassembly": bool(is_subasm),
                "depth": depth,
                "transform_4x4_local": t_local,
                "transform2_4x4_local": t2_local,
                "world_position_xyz_cm": world_xyz,
                "world_rotation_3x3": world_rot,
            }
        except Exception as e:
            out["occurrences"][getattr(occ, 'fullPathName', 'unknown')] = {
                "error": str(e),
            }

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2)

    return len(out["occurrences"])
