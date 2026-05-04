"""
User config loader -- reads ``xacro_export.toml`` from the plugin
folder and merges its values into ``ExportConfig`` defaults.

Why TOML?  Plain text, comment-friendly, and Python 3.11+ ships a
parser in the stdlib (``tomllib``).  For older Python (Fusion's
bundled interpreter on some versions) we fall back to a minimal
hand-written parser that handles the schema this plugin needs:

  [section]
  key = value          # comment

  values: "string", 'string', true/false, integer, float

No nested arrays-of-tables, no inline tables, no datetime types.
Anything more elaborate belongs in a real TOML library.

If the config file doesn't exist the plugin uses ``ExportConfig``'s
hardcoded defaults -- running without a config is fine.

Author: Adrian Valaker Eikeland
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional


def _try_stdlib_tomllib():
    """Return ``tomllib.loads`` if available (Python 3.11+), else None."""
    try:
        import tomllib  # type: ignore
        return tomllib.loads
    except Exception:
        return None


def _parse_toml_minimal(text: str) -> Dict[str, Dict[str, Any]]:
    """Minimal TOML parser sufficient for the export config schema.

    Supports section headers, ``key = value`` lines, string / bool /
    int / float values, and ``#`` comments.  Anything outside that is
    ignored or raises ValueError.

    Returns nested dict: ``{section_name: {key: value}}``.  Top-level
    keys (no section) live in the ``""`` (empty string) section.
    """
    out: Dict[str, Dict[str, Any]] = {"": {}}
    section = ""
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        # Strip trailing comment (only if not inside a string)
        if '#' in line:
            in_str = False
            quote = ''
            for i, ch in enumerate(line):
                if not in_str and ch in ('"', "'"):
                    in_str = True
                    quote = ch
                elif in_str and ch == quote:
                    in_str = False
                elif not in_str and ch == '#':
                    line = line[:i].rstrip()
                    break

        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1].strip()
            out.setdefault(section, {})
            continue

        if '=' not in line:
            continue
        key, _, raw_val = line.partition('=')
        key = key.strip()
        raw_val = raw_val.strip()

        # Parse value
        if (raw_val.startswith('"') and raw_val.endswith('"')) or \
           (raw_val.startswith("'") and raw_val.endswith("'")):
            value: Any = raw_val[1:-1]
        elif raw_val.lower() == 'true':
            value = True
        elif raw_val.lower() == 'false':
            value = False
        else:
            try:
                value = int(raw_val)
            except ValueError:
                try:
                    value = float(raw_val)
                except ValueError:
                    # Bare string fallback — TOML proper would reject
                    # this, but we accept it for resilience.
                    value = raw_val
        out[section][key] = value
    return out


def load_toml(path: str) -> Dict[str, Dict[str, Any]]:
    """Read and parse a TOML file.  Returns the section→key→value dict.

    Empty result if the file doesn't exist or fails to parse — the
    caller should treat that the same as "use defaults."
    """
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()
    except Exception:
        return {}

    parser = _try_stdlib_tomllib()
    if parser is not None:
        try:
            data = parser(text)
            # tomllib gives nested dict already; flatten so top-level
            # keys live under "" to match the minimal parser shape.
            normalized: Dict[str, Dict[str, Any]] = {"": {}}
            for k, v in data.items():
                if isinstance(v, dict):
                    normalized[k] = v
                else:
                    normalized[""][k] = v
            return normalized
        except Exception:
            pass  # fall back to minimal parser
    try:
        return _parse_toml_minimal(text)
    except Exception:
        return {}


# ──────────────────────────────────────────────
# Default config search path
# ──────────────────────────────────────────────

CONFIG_FILENAME = "xacro_export.toml"


def _config_candidates(plugin_root: str):
    """Return config paths in lookup order for ``plugin_root``.

    The plugin-local file is the only supported persistent config path.
    """
    return (
        os.path.join(plugin_root, CONFIG_FILENAME),
    )


def find_config_path() -> Optional[str]:
    """Locate ``xacro_export.toml`` in the plugin folder.

    Returns ``None`` when the plugin-local file is absent.
    """
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for candidate in _config_candidates(here):
        if os.path.isfile(candidate):
            return candidate
    return None


# ──────────────────────────────────────────────
# Apply to ExportConfig
# ──────────────────────────────────────────────

# Verbosity → which outputs are on by default.
_VERBOSE_DEFAULTS = {
    "include_debug": True,
    "include_docs": True,
    "include_robot_data_yaml": True,
    "include_screenshot": True,
    "include_launch": True,
    "include_rviz": True,
    "include_readme": True,
    "include_ros2_control": True,
}
_MINIMAL_DEFAULTS = {
    "include_debug": False,
    "include_docs": False,
    "include_robot_data_yaml": False,
    "include_screenshot": False,
    "include_launch": True,    # one launch file is small and useful
    "include_rviz": False,
    "include_readme": False,
    "include_ros2_control": False,
}


def apply_to_config(cfg, toml_data: Dict[str, Dict[str, Any]]) -> list:
    """Mutate ``cfg`` (an ``ExportConfig``) in-place from parsed TOML.

    Returns a list of human-readable changes for logging.

    Section layout the loader recognises::

        [output]
        verbosity = "verbose"   # or "minimal"
        zip = false
        zip_name = ""

        [features]
        include_debug = true
        include_docs = true
        include_robot_data_yaml = true
        include_screenshot = true
        include_launch = true
        include_rviz = true
        include_readme = true
        include_ros2_control = true

        [mesh]
        visual_format = "dae"   # or "obj"
        collision_method = "primitive"   # or "convex_hull" / "visual_reuse"
        mesh_refinement = "medium"       # low / medium / high

        [ros2_control]
        hardware_plugin = "mock_components/GenericSystem"
        update_rate = 100
        command_interfaces = "position,velocity"

    Unknown keys are ignored (logged as informational, not errors)
    so a config from a future plugin version doesn't blow up an
    older install.
    """
    changes: list = []
    output = toml_data.get("output", {}) or {}
    features = toml_data.get("features", {}) or {}
    mesh = toml_data.get("mesh", {}) or {}
    ros2_control = toml_data.get("ros2_control", {}) or {}

    # Verbosity drives default include_* values; explicit [features]
    # entries override per-key.
    verbosity = output.get("verbosity")
    if isinstance(verbosity, str):
        verbosity = verbosity.lower().strip()
        if verbosity == "minimal":
            for k, v in _MINIMAL_DEFAULTS.items():
                setattr(cfg, k, v)
            changes.append("verbosity=minimal: most optional outputs OFF")
        elif verbosity == "verbose":
            for k, v in _VERBOSE_DEFAULTS.items():
                setattr(cfg, k, v)
            changes.append("verbosity=verbose: all optional outputs ON")
        cfg.verbosity = verbosity

    for k in _VERBOSE_DEFAULTS.keys():
        if k in features and isinstance(features[k], bool):
            setattr(cfg, k, features[k])
            changes.append(f"{k} = {features[k]}")

    if "zip" in output and isinstance(output["zip"], bool):
        cfg.zip_output = output["zip"]
        changes.append(f"zip_output = {output['zip']}")
    if "zip_name" in output and isinstance(output["zip_name"], str):
        cfg.zip_name = output["zip_name"]

    if "visual_format" in mesh and isinstance(mesh["visual_format"], str):
        cfg.visual_format = mesh["visual_format"].lower().strip()
        changes.append(f"visual_format = {cfg.visual_format}")
    if "collision_method" in mesh and isinstance(mesh["collision_method"], str):
        cfg.collision_auto_method = mesh["collision_method"].lower().strip().replace("-", "_")
        changes.append(f"collision_method = {cfg.collision_auto_method}")
    if "mesh_refinement" in mesh and isinstance(mesh["mesh_refinement"], str):
        cfg.mesh_refinement = mesh["mesh_refinement"].lower().strip()
        changes.append(f"mesh_refinement = {cfg.mesh_refinement}")

    if "hardware_plugin" in ros2_control and isinstance(ros2_control["hardware_plugin"], str):
        cfg.ros2_control_hardware_plugin = ros2_control["hardware_plugin"].strip()
        changes.append(f"ros2_control.hardware_plugin = {cfg.ros2_control_hardware_plugin}")
    if "update_rate" in ros2_control and isinstance(ros2_control["update_rate"], (int, float)):
        cfg.ros2_control_update_rate = int(ros2_control["update_rate"])
        changes.append(f"ros2_control.update_rate = {cfg.ros2_control_update_rate}")
    if "command_interfaces" in ros2_control:
        raw = ros2_control["command_interfaces"]
        if isinstance(raw, str):
            interfaces = [item.strip().lower() for item in raw.split(",")]
        elif isinstance(raw, (list, tuple)):
            interfaces = [str(item).strip().lower() for item in raw]
        else:
            interfaces = []
        interfaces = [
            name for name in interfaces
            if name in ("position", "velocity", "effort")
        ]
        if interfaces:
            cfg.ros2_control_command_interfaces = tuple(dict.fromkeys(interfaces))
            changes.append(
                "ros2_control.command_interfaces = "
                f"{','.join(cfg.ros2_control_command_interfaces)}"
            )

    return changes
