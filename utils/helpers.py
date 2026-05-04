"""
Utilities — Shared helper functions for the Fusion URDF Exporter.

Single source of truth for:
  - Name cleaning (component names → URDF-safe names)
  - Unit conversion (Fusion cm → URDF meters, etc.)
  - Float precision cleaning
  - Serialization helpers (dataclass → dict for JSON)

Author: Adrian Valaker Eikeland
"""

import re
import json
from dataclasses import asdict, is_dataclass
from typing import Any


# ──────────────────────────────────────────────
# Name cleaning
# ──────────────────────────────────────────────

def clean_name(name: str) -> str:
    """
    Clean Fusion component name for URDF compatibility.
    
    Removes version numbers, occurrence IDs, and special characters.
    
    Examples:
        "head_link v8"     → "head_link"
        "turret v25:1"     → "turret"
        "My Part (v2):1"   → "My_Part"
        "neck_link v6"     → "neck_link"
    """
    if not name:
        return ""
    
    # Strip version numbers: " v18", " v25.1", etc.
    clean = re.sub(r'\s+v\d+(\.\d+)*', '', name)
    # Strip occurrence IDs: ":1", ":2", etc.
    clean = re.sub(r':\d+$', '', clean)
    # Replace non-alphanumeric (except underscore) with underscore
    clean = re.sub(r'[^a-zA-Z0-9_]', '_', clean)
    # Collapse multiple underscores
    clean = re.sub(r'_+', '_', clean)
    # Strip leading/trailing underscores
    clean = clean.strip('_')
    
    return clean


SPECIAL_PREFIX_MARKER = "!"


def yaml_safe_name(name: str) -> str:
    """Strip a leading ``!`` so a Fusion-source name is safe to write
    into YAML without quoting.

    YAML treats a leading ``!`` as the start of a tag (e.g. ``!!str``,
    ``!Custom``).  An unquoted scalar like ``!frame_base_link`` makes
    the whole document unparseable by ``yaml.safe_load``.

    Most names that reach generated artifacts have already been
    stripped of their reserved prefix (``!frame_*``, ``!collision_*``,
    ``!passive_*`` etc.) by the various ``strip_*`` helpers above.
    This guard exists for places where the *raw* Fusion source name
    is intentionally preserved — most notably the ``merged_from``
    traceability list in ``robot_data.yaml``.

    Returns ``name`` unchanged when there is no leading ``!``.
    """
    if not name:
        return name
    return name[1:] if name.startswith(SPECIAL_PREFIX_MARKER) else name



METADATA_PREFIXES = {
    "collision": {"kind": "collision"},
    "acc": {"kind": "collision_override", "method": "visual"},
    "cxh": {"kind": "collision_override", "method": "convex_hull"},
    "pri": {"kind": "collision_override", "method": "primitive"},
    "frame": {"kind": "frame_only"},
    "dummy": {"kind": "dummy"},
    "passive": {"kind": "joint", "flag": "passive"},
    "closing": {"kind": "joint", "flag": "closing"},
}


def dispatch_metadata_prefix(name: str, allowed=None) -> dict:
    """Classify a Fusion name that may start with a reserved metadata prefix.

    Metadata keywords must use the explicit ``!`` marker, e.g.
    ``!cxh_body`` or ``!frame_imu``. Keywords only match when they are
    bare or followed by a separator, so names like ``!accelerometer`` are
    not treated as ``!acc`` metadata.

    Returns a small dispatch record:
      - ``keyword``: matched reserved keyword, or ``""``.
      - ``kind``: semantic family (``collision_override``, ``frame_only``,
        ``joint`` ...), or ``""``.
      - ``remainder``: name after the keyword marker/separator.
      - ``tagged``: whether the input used the explicit ``!`` marker.
      - optional metadata such as ``method`` or ``flag``.
    """
    empty = {
        "keyword": "",
        "kind": "",
        "remainder": name or "",
        "tagged": False,
        "method": "",
        "flag": "",
    }
    if not name:
        return empty

    text = name.strip()
    tagged = text.startswith(SPECIAL_PREFIX_MARKER)
    if not tagged:
        return empty

    body = text[1:].lstrip()
    lowered = body.lower()
    allowed_keywords = (
        {keyword.lower() for keyword in allowed}
        if allowed is not None else None
    )

    for keyword, metadata in METADATA_PREFIXES.items():
        if allowed_keywords is not None and keyword not in allowed_keywords:
            continue
        remainder = None
        if lowered == keyword:
            remainder = ""
        else:
            for sep in ("_", "-", " "):
                marker = keyword + sep
                if lowered.startswith(marker):
                    remainder = body[len(marker):].lstrip()
                    break
        if remainder is None:
            continue

        result = dict(empty)
        result.update(metadata)
        result.update({
            "keyword": keyword,
            "remainder": remainder,
            "tagged": tagged,
        })
        return result

    result = dict(empty)
    result["tagged"] = tagged
    return result


def parse_collision_override_prefix(name: str) -> tuple:
    """Parse per-link collision override metadata from a Fusion name.

    Returns ``(method, stripped_name)`` where method is one of:
    ``"visual"`` for ``acc``, ``"convex_hull"`` for ``cxh``, or
    ``"primitive"`` for ``pri``.  ``("", name)`` means no override.
    """
    meta = dispatch_metadata_prefix(name, ("acc", "cxh", "pri"))
    if meta["kind"] != "collision_override":
        return "", name
    return meta["method"], meta["remainder"]


def strip_collision_override_prefix(name: str) -> str:
    """Strip any collision override prefix, preserving the input if absent."""
    method, stripped = parse_collision_override_prefix(name)
    return stripped if method else name


def is_accurate_collision_group_name(name: str) -> bool:
    """True if a rigid-group (or single-component link) name is tagged
    ``!acc_*`` to mean 'use the visual mesh as collision geometry instead
    of the auto-fitted primitive'.  Useful for parts that need accurate
    contact (e.g. a gripper jaw with grooves) while the rest of the robot
    stays cheap with bounding-primitive collision.
    """
    method, _stripped = parse_collision_override_prefix(name)
    return method == "visual"


def strip_accurate_collision_prefix(name: str) -> str:
    """Strip the ``!acc_`` metadata prefix.

    Bare ``!acc`` returns ``""``; callers should fall back to the
    original name if that is not useful.
    """
    method, stripped = parse_collision_override_prefix(name)
    return stripped if method == "visual" else name


def is_frame_only_name(name: str) -> bool:
    """True when a component name marks a pure attachment frame.

    ``!frame_*`` components become URDF links with no visual, collision, or
    inertial data.
    """
    return dispatch_metadata_prefix(name, ("frame",))["kind"] == "frame_only"


def strip_frame_prefix(name: str) -> str:
    """Strip the ``!frame_`` metadata prefix from a component name."""
    meta = dispatch_metadata_prefix(name, ("frame",))
    return meta["remainder"] if meta["kind"] == "frame_only" else name


def is_dummy_assembly_name(name: str) -> bool:
    """True when an assembly/component name marks a swappable dummy module."""
    return dispatch_metadata_prefix(name, ("dummy",))["kind"] == "dummy"


def strip_link_metadata_prefixes(name: str) -> tuple:
    """Strip one collision override prefix and one frame prefix from a link.

    Returns ``(clean_source_name, collision_override, is_frame_only)``.
    Prefix order is intentionally limited: each convention consumes at most
    one leading tag, so accidental stacked tags remain visible in diagnostics
    instead of silently disappearing.
    """
    working = name or ""
    collision_override, stripped = parse_collision_override_prefix(working)
    if collision_override:
        working = stripped or working

    is_frame = is_frame_only_name(working)
    if is_frame:
        stripped = strip_frame_prefix(working)
        working = stripped or working

    return working, collision_override, is_frame


def clean_link_name(name: str) -> str:
    """Clean a Fusion link/component name after stripping metadata tags."""
    stripped, _collision_override, _is_frame = strip_link_metadata_prefixes(name)
    cleaned = clean_name(stripped)
    return cleaned or clean_name(name)


def safe_identifier(name: str, fallback: str = "unnamed") -> str:
    """Sanitize an arbitrary string into a USD/URDF-safe identifier.

    Both URDF (``<name>`` attributes referenced as XML IDs) and USD
    (``Sdf.Path`` segments) require ASCII alphanumerics + underscore
    and reject leading digits.  Non-ASCII characters (Chinese, Cyrillic,
    accented Latin) crash Isaac Sim's URDF importer with
    ``LLVM ERROR: out of memory`` after a half-attempted rewrite.

    Use for any string that flows into:
      - URDF link / joint / material names,
      - USD prim path segments,
      - mesh material names embedded in OBJ/MTL/DAE.

    ``clean_name`` is the right helper for *component* names (it also
    strips Fusion's ``" v1"`` / ``":1"`` suffixes); ``safe_identifier``
    is the right helper for downloaded-asset metadata (material names,
    appearance names) where the suffix-stripping is wrong.

    Returns ``fallback`` (default ``"unnamed"``) when sanitization
    yields an empty string — better than emitting an empty XML
    attribute or a bare ``_`` USD path segment.
    """
    if not name:
        return fallback
    s = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    s = re.sub(r'_+', '_', s).strip('_')
    if not s:
        return fallback
    if s[0].isdigit():
        s = '_' + s
    return s


def is_collision_component_name(name: str) -> bool:
    """True if a component's clean name marks it as collision geometry.

    Accepts ``!collision`` and ``!collision_<suffix>``. Component names
    without ``!`` are normal visual/link names.
    """
    return dispatch_metadata_prefix(name, ("collision",))["kind"] == "collision"


def is_collision_body_name(name: str) -> bool:
    """True if a body name marks explicit collision geometry.

    Body names must be authored as ``!collision_*``. Fusion duplicate
    suffixes such as ``!collision (1)`` still match because the bang
    marker remains present.
    """
    return dispatch_metadata_prefix(name, ("collision",))["kind"] == "collision"


def is_collision_excluded_body_name(name: str) -> bool:
    """True if a visual body should be ignored by generated collision.

    Body-level ``!`` is intentionally lightweight: naming a body
    ``!antenna`` keeps it in the visual mesh, but removes it from generated
    primitive and convex-hull collision input.  Reserved body tags such as
    ``!collision_*`` keep their normal meaning and are not treated as visual
    collision exclusions.
    """
    meta = dispatch_metadata_prefix(name)
    return bool(meta["tagged"] and not meta["kind"])


def is_passive_joint_name(name: str) -> bool:
    """True if a joint's name marks it as passive (no controller drive).

    Accepts ``!passive_<suffix>``. Bare ``!passive`` is also accepted
    but rare in practice. Joints flagged passive get
    ``drive_type: none`` in the downstream pipeline (no DriveAPI applied
    in USD; no controller in ROS).  Idler wheels, free pivots, and the
    follower joints in closed kinematic chains are typical passive
    joints.
    """
    meta = dispatch_metadata_prefix(name, ("passive",))
    return meta["kind"] == "joint" and meta["flag"] == "passive"


def is_closing_joint_name(name: str) -> bool:
    """True if a joint's name marks it as a closing-loop joint.

    Accepts ``!closing_<suffix>``. Closing joints are excluded from the URDF tree
    (URDF can't represent cycles) and emitted to the ``closing_joints:``
    section of
    ``robot_data.yaml`` for downstream tools that can author them
    (e.g. URDF→USD pipelines, where they become independent
    ``UsdPhysicsJoint`` prims that close the loop).  Closing joints
    are implicitly passive.
    """
    meta = dispatch_metadata_prefix(name, ("closing",))
    return meta["kind"] == "joint" and meta["flag"] == "closing"


def strip_joint_prefix(name: str):
    """Recognise + strip ``!passive_`` / ``!closing_`` prefixes from a
    joint name.

    Returns ``(cleaned_name, is_passive, is_closing)``.  The cleaned
    name has the prefix removed; the boolean flags are independently
    derived (a joint can be both, e.g. ``!passive_closing_left_slider``
    would resolve to ``("left_slider", True, True)``).

    A closing joint is implicitly passive, so callers can usually
    treat ``is_closing`` as implying ``is_passive`` regardless of
    whether the user spelled both prefixes.
    """
    if not name:
        return "", False, False
    cleaned = name
    is_closing = False
    is_passive = False
    # Strip up to one of each prefix in either order so users can
    # write ``!closing_passive_x`` or ``!passive_closing_x`` and get
    # the same result.
    for _ in range(2):
        meta = dispatch_metadata_prefix(cleaned, ("closing", "passive"))
        if meta["kind"] != "joint":
            break
        if meta["flag"] == "closing":
            is_closing = True
            cleaned = meta["remainder"]
            continue
        if meta["flag"] == "passive":
            is_passive = True
            cleaned = meta["remainder"]
            continue
    if is_closing:
        is_passive = True  # closing implies passive
    return cleaned, is_passive, is_closing


def explicit_collision_names(snapshot) -> list:
    """Clean names of occurrences and rigid-group members that qualify as
    explicit collision geometry.  Returns deduplicated, sorted list.

    Used by the export UI to tell the user which components will be
    treated as explicit collision when they pick a simple primary
    collision method.
    """
    names = {
        occ.clean_name
        for occ in snapshot.occurrences.values()
        if getattr(occ, "is_collision_geometry", False)
    }
    names.update(
        rg.collision_member
        for rg in snapshot.rigid_groups
        if rg.collision_member
    )
    return sorted(names)


def parse_occurrence_path(full_path: str):
    """
    Parse a Fusion occurrence fullPathName into segments.
    
    "turret v25:1+dummy_zed2i v4:1+zed2i_link v5:1"
    → ["turret", "dummy_zed2i", "zed2i_link"]
    
    "base_link v5:1"
    → ["base_link"]
    
    Returns:
        List of clean segment names
    """
    if not full_path:
        return []
    
    raw_segments = full_path.split('+')
    return [clean_link_name(seg) for seg in raw_segments if seg.strip()]


# ──────────────────────────────────────────────
# Unit conversion
# ──────────────────────────────────────────────

def cm_to_m(cm: float) -> float:
    """Centimeters → meters."""
    return cm / 100.0

def mm_to_m(mm: float) -> float:
    """Millimeters → meters."""
    return mm / 1000.0

def cm2_to_m2(cm2: float) -> float:
    """cm² → m²."""
    return cm2 / 1e4

def cm3_to_m3(cm3: float) -> float:
    """cm³ → m³."""
    return cm3 / 1e6

def mm3_to_m3(mm3: float) -> float:
    """mm³ → m³."""
    return mm3 / 1e9

def g_to_kg(g: float) -> float:
    """Grams → kilograms."""
    return g / 1000.0

def g_per_cm3_to_kg_per_m3(g_cm3: float) -> float:
    """g/cm³ → kg/m³."""
    return g_cm3 * 1000.0

def g_mm2_to_kg_m2(g_mm2: float) -> float:
    """g·mm² → kg·m² (inertia unit from Fusion Properties panel)."""
    return g_mm2 / 1e9

def kg_cm2_to_kg_m2(kg_cm2: float) -> float:
    """kg·cm² → kg·m² (inertia unit from Fusion API)."""
    return kg_cm2 / 1e4


# ──────────────────────────────────────────────
# Float precision
# ──────────────────────────────────────────────

def epsilon_clean(value: float, eps: float = 1e-10) -> float:
    """Zero out values smaller than epsilon."""
    return 0.0 if abs(value) < eps else value

def clean_vec3(xyz, eps: float = 1e-10):
    """Epsilon-clean a 3-tuple."""
    return tuple(epsilon_clean(v, eps) for v in xyz)

def fmt(value: float, decimals: int = 6) -> str:
    """Format float for URDF output. Clean epsilon noise, strip trailing zeros."""
    cleaned = epsilon_clean(value)
    formatted = f"{cleaned:.{decimals}f}"
    # Strip trailing zeros but keep at least one decimal
    if '.' in formatted:
        formatted = formatted.rstrip('0').rstrip('.')
        if '.' not in formatted:
            formatted += '.0'
    return formatted

def fmt_vec3(xyz, decimals: int = 6) -> str:
    """Format Vec3 as space-separated string for URDF."""
    return " ".join(fmt(v, decimals) for v in xyz)


# ──────────────────────────────────────────────
# Serialization
# ──────────────────────────────────────────────

class DataclassEncoder(json.JSONEncoder):
    """JSON encoder that handles dataclasses and tuples."""
    
    def default(self, obj):
        if is_dataclass(obj) and not isinstance(obj, type):
            d = {}
            for k, v in obj.__dict__.items():
                # Skip private fields (Fusion references)
                if k.startswith('_'):
                    continue
                d[k] = v
            return d
        if isinstance(obj, tuple):
            return list(obj)
        return super().default(obj)


def snapshot_to_json(snapshot, indent: int = 2) -> str:
    """Serialize a FusionSnapshot to JSON string."""
    return json.dumps(snapshot, cls=DataclassEncoder, indent=indent)


def snapshot_to_file(snapshot, path: str):
    """Save FusionSnapshot as JSON file."""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(snapshot_to_json(snapshot))
