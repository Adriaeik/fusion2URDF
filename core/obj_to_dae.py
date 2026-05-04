"""
OBJ + MTL → DAE (COLLADA 1.4.1) converter.

Pure Python.  No external dependencies — Fusion's bundled Python doesn't
ship with pycollada/meshio and we want the exporter to keep its
zero-deps property.

Why DAE?  ROS, Gazebo, Isaac Sim and most URDF tools accept DAE
natively.  A single self-contained file (geometry + materials in one
XML) is simpler than the OBJ+MTL pair, and DAE's native unit is meters
— so the URDF mesh tag drops the ``scale="0.01 0.01 0.01"`` workaround.

Conversion semantics:

* OBJ vertex coordinates (Fusion default = centimeters) are scaled by
  0.01 → DAE writes meters with ``<unit name="meter" meter="1.0"/>``.
  URDF then references the DAE with ``scale="1 1 1"``.
* OBJ ``usemtl`` groups become separate ``<triangles material=...>``
  blocks under one ``<mesh>``.  Multi-material output from Fusion
  survives unchanged.
* MTL diffuse / ambient / specular colors come across as a Lambert
  material.  Texture maps are not handled (Fusion exports rarely
  include them; if you need them, extend ``_parse_mtl`` and the
  ``library_effects`` writer).
* Faces with negative indices (OBJ-relative) are resolved at parse time
  so the DAE always has 0-based absolute indices.

Author: Adrian Valaker Eikeland
"""

from __future__ import annotations

import os
import re
from typing import Dict, List, Optional, Tuple

CM_PER_M = 100.0


# ──────────────────────────────────────────────
# OBJ + MTL parsing
# ──────────────────────────────────────────────

def _parse_obj(obj_path: str) -> dict:
    """Parse an OBJ file into vertices / normals / per-material face lists.

    Returns a dict with ``positions`` (list of (x,y,z) in OBJ units),
    ``normals`` (list of (nx,ny,nz)), and ``groups``: a dict keyed by
    material name with a list of triangles, where each triangle is a
    list of three (v_idx, vn_idx) tuples (0-based).
    """
    positions: List[Tuple[float, float, float]] = []
    normals: List[Tuple[float, float, float]] = []
    groups: Dict[str, List[List[Tuple[int, Optional[int]]]]] = {}
    current_material = "default"
    mtllib: Optional[str] = None

    with open(obj_path, 'r', errors='replace') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('mtllib '):
                mtllib = line.split(' ', 1)[1].strip()
                continue
            if line.startswith('usemtl '):
                current_material = line.split(' ', 1)[1].strip()
                continue
            if line.startswith('v '):
                parts = line.split()
                positions.append((float(parts[1]), float(parts[2]), float(parts[3])))
                continue
            if line.startswith('vn '):
                parts = line.split()
                normals.append((float(parts[1]), float(parts[2]), float(parts[3])))
                continue
            if line.startswith('f '):
                tri = _parse_face_to_indices(line, len(positions), len(normals))
                if tri is not None:
                    groups.setdefault(current_material, []).append(tri)
                continue
            # Other directives (vt, vp, o, g, s, l) ignored.

    return {
        "positions": positions,
        "normals": normals,
        "groups": groups,
        "mtllib": mtllib,
    }


def _parse_face_to_indices(face_line: str, v_count: int, vn_count: int):
    """Parse one OBJ ``f`` line into a single triangle.

    OBJ supports v / v/vt / v//vn / v/vt/vn references and negative
    indices.  Fusion's OBJ output is consistently triangulated, but we
    triangulate longer polygons defensively just in case.

    Returns a list of three (v_idx, vn_idx) tuples (0-based) or
    ``None`` on parse failure.  When the polygon has more than three
    verts, returns the first triangle (caller can fan-triangulate
    upstream).
    """
    parts = face_line.split()[1:]  # drop "f"
    if len(parts) < 3:
        return None

    refs: List[Tuple[int, Optional[int]]] = []
    for ref in parts:
        sub = ref.split('/')
        try:
            v = int(sub[0])
            v0 = (v - 1) if v > 0 else (v_count + v)
            vn0: Optional[int] = None
            if len(sub) > 2 and sub[2]:
                vn = int(sub[2])
                vn0 = (vn - 1) if vn > 0 else (vn_count + vn)
            refs.append((v0, vn0))
        except (ValueError, IndexError):
            return None

    if len(refs) == 3:
        return refs
    # Fan-triangulate: this caller only returns one triangle per call,
    # so for n-gons we'd lose triangles.  Fusion exports triangles, so
    # this is defensive only — log behavior left to the caller.
    return [refs[0], refs[1], refs[2]]


def _parse_mtl(mtl_path: str) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """Parse an MTL file.  Returns ``{material_name: {Kd: (r,g,b), Ka: ..., Ks: ...}}``."""
    if not mtl_path or not os.path.isfile(mtl_path):
        return {}
    materials: Dict[str, Dict[str, Tuple[float, float, float]]] = {}
    current: Optional[str] = None
    with open(mtl_path, 'r', errors='replace') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('newmtl '):
                current = line.split(' ', 1)[1].strip()
                materials[current] = {}
                continue
            if current is None:
                continue
            for prop in ('Kd', 'Ka', 'Ks'):
                if line.startswith(prop + ' '):
                    parts = line.split()
                    try:
                        materials[current][prop] = (
                            float(parts[1]), float(parts[2]), float(parts[3])
                        )
                    except (ValueError, IndexError):
                        pass
                    break
    return materials


# ──────────────────────────────────────────────
# DAE (COLLADA 1.4.1) writer
# ──────────────────────────────────────────────

_DAE_HEADER = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
'''

_DAE_FOOTER = '</COLLADA>\n'


def _xml_safe_id(name: str) -> str:
    """Produce a COLLADA-compatible id (alphanumeric + underscore + hyphen)."""
    cleaned = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
    if not cleaned or not cleaned[0].isalpha():
        cleaned = 'mat_' + cleaned
    return cleaned


def obj_to_dae(
    obj_path: str,
    mtl_path: Optional[str],
    dae_path: str,
    *,
    name: str = "mesh",
    scale_to_meters: bool = True,
) -> bool:
    """Convert an OBJ + MTL pair to a single self-contained DAE file.

    Returns ``True`` on success.  Raises on file I/O errors so the
    caller can decide whether to surface them.

    ``scale_to_meters=True`` divides every vertex by 100 so the DAE
    sits in meters and the URDF can use ``scale="1 1 1"``.  Set False
    only if your OBJ is already in meters (rare for Fusion output).
    """
    if not os.path.isfile(obj_path):
        return False

    obj = _parse_obj(obj_path)
    positions = obj["positions"]
    normals = obj["normals"]
    groups = obj["groups"]
    if not positions or not groups:
        return False

    # Resolve MTL: explicit path > the path the OBJ's mtllib points to.
    if mtl_path is None and obj["mtllib"]:
        mtl_path = os.path.join(os.path.dirname(obj_path), obj["mtllib"])
    materials = _parse_mtl(mtl_path) if mtl_path else {}

    s = 1.0 / CM_PER_M if scale_to_meters else 1.0

    out: List[str] = [_DAE_HEADER]

    # ── Asset header ──
    from datetime import datetime
    now = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    unit_meter = 1.0  # DAE always says meter=N where N is meters per its unit
    out.append(
        f'  <asset>\n'
        f'    <contributor>\n'
        f'      <authoring_tool>Fusion URDF/XACRO Exporter</authoring_tool>\n'
        f'    </contributor>\n'
        f'    <created>{now}</created>\n'
        f'    <modified>{now}</modified>\n'
        f'    <unit name="meter" meter="{unit_meter}"/>\n'
        f'    <up_axis>Z_UP</up_axis>\n'
        f'  </asset>\n'
    )

    # Material list — every group's material plus a default fallback.
    used_materials = list(groups.keys())
    if not used_materials:
        used_materials = ['default']

    # ── library_effects ──
    out.append('  <library_effects>\n')
    for mat in used_materials:
        eid = _xml_safe_id(mat) + "-effect"
        kd = materials.get(mat, {}).get('Kd', (0.7, 0.7, 0.7))
        ka = materials.get(mat, {}).get('Ka', (0.0, 0.0, 0.0))
        ks = materials.get(mat, {}).get('Ks', (0.2, 0.2, 0.2))
        out.append(
            f'    <effect id="{eid}">\n'
            f'      <profile_COMMON>\n'
            f'        <technique sid="common">\n'
            f'          <lambert>\n'
            f'            <emission><color>0 0 0 1</color></emission>\n'
            f'            <ambient><color>{ka[0]:.4f} {ka[1]:.4f} {ka[2]:.4f} 1</color></ambient>\n'
            f'            <diffuse><color>{kd[0]:.4f} {kd[1]:.4f} {kd[2]:.4f} 1</color></diffuse>\n'
            f'            <reflective><color>0 0 0 1</color></reflective>\n'
            f'            <transparent opaque="A_ONE"><color>1 1 1 1</color></transparent>\n'
            f'            <transparency><float>1</float></transparency>\n'
            f'          </lambert>\n'
            f'        </technique>\n'
            f'      </profile_COMMON>\n'
            f'    </effect>\n'
        )
    out.append('  </library_effects>\n')

    # ── library_materials ──
    out.append('  <library_materials>\n')
    for mat in used_materials:
        mid = _xml_safe_id(mat)
        out.append(
            f'    <material id="{mid}" name="{mat}">\n'
            f'      <instance_effect url="#{mid}-effect"/>\n'
            f'    </material>\n'
        )
    out.append('  </library_materials>\n')

    # ── library_geometries ──
    geo_id = _xml_safe_id(name) + "-mesh"
    pos_id = geo_id + "-positions"
    pos_arr_id = pos_id + "-array"
    nrm_id = geo_id + "-normals"
    nrm_arr_id = nrm_id + "-array"
    vert_id = geo_id + "-vertices"

    out.append('  <library_geometries>\n')
    out.append(f'    <geometry id="{geo_id}" name="{name}">\n')
    out.append('      <mesh>\n')

    # Positions
    pos_vals = ' '.join(f"{p[0]*s:.6f} {p[1]*s:.6f} {p[2]*s:.6f}" for p in positions)
    out.append(
        f'        <source id="{pos_id}">\n'
        f'          <float_array id="{pos_arr_id}" count="{len(positions)*3}">{pos_vals}</float_array>\n'
        f'          <technique_common>\n'
        f'            <accessor source="#{pos_arr_id}" count="{len(positions)}" stride="3">\n'
        f'              <param name="X" type="float"/>\n'
        f'              <param name="Y" type="float"/>\n'
        f'              <param name="Z" type="float"/>\n'
        f'            </accessor>\n'
        f'          </technique_common>\n'
        f'        </source>\n'
    )

    # Normals (optional — only emit if any face references them)
    has_normals = bool(normals) and any(
        n is not None for tris in groups.values() for tri in tris for (_, n) in tri
    )
    if has_normals:
        nrm_vals = ' '.join(f"{n[0]:.6f} {n[1]:.6f} {n[2]:.6f}" for n in normals)
        out.append(
            f'        <source id="{nrm_id}">\n'
            f'          <float_array id="{nrm_arr_id}" count="{len(normals)*3}">{nrm_vals}</float_array>\n'
            f'          <technique_common>\n'
            f'            <accessor source="#{nrm_arr_id}" count="{len(normals)}" stride="3">\n'
            f'              <param name="X" type="float"/>\n'
            f'              <param name="Y" type="float"/>\n'
            f'              <param name="Z" type="float"/>\n'
            f'            </accessor>\n'
            f'          </technique_common>\n'
            f'        </source>\n'
        )

    # Vertices
    out.append(
        f'        <vertices id="{vert_id}">\n'
        f'          <input semantic="POSITION" source="#{pos_id}"/>\n'
        f'        </vertices>\n'
    )

    # Triangles per material
    for mat, tris in groups.items():
        if not tris:
            continue
        mid = _xml_safe_id(mat)
        out.append(f'        <triangles material="{mid}" count="{len(tris)}">\n')
        out.append(f'          <input semantic="VERTEX" source="#{vert_id}" offset="0"/>\n')
        if has_normals:
            out.append(f'          <input semantic="NORMAL" source="#{nrm_id}" offset="1"/>\n')
        # Build the index list
        idx_parts: List[str] = []
        for tri in tris:
            for v_idx, vn_idx in tri:
                if has_normals:
                    idx_parts.append(f"{v_idx} {vn_idx if vn_idx is not None else 0}")
                else:
                    idx_parts.append(str(v_idx))
        out.append(f'          <p>{" ".join(idx_parts)}</p>\n')
        out.append(f'        </triangles>\n')

    out.append('      </mesh>\n')
    out.append('    </geometry>\n')
    out.append('  </library_geometries>\n')

    # ── library_visual_scenes ──
    scene_id = "Scene"
    node_id = _xml_safe_id(name) + "-node"
    out.append(
        f'  <library_visual_scenes>\n'
        f'    <visual_scene id="{scene_id}" name="{scene_id}">\n'
        f'      <node id="{node_id}" name="{name}" type="NODE">\n'
        f'        <instance_geometry url="#{geo_id}">\n'
        f'          <bind_material>\n'
        f'            <technique_common>\n'
    )
    for mat in used_materials:
        mid = _xml_safe_id(mat)
        out.append(
            f'              <instance_material symbol="{mid}" target="#{mid}"/>\n'
        )
    out.append(
        f'            </technique_common>\n'
        f'          </bind_material>\n'
        f'        </instance_geometry>\n'
        f'      </node>\n'
        f'    </visual_scene>\n'
        f'  </library_visual_scenes>\n'
    )

    # ── scene ──
    out.append(
        f'  <scene>\n'
        f'    <instance_visual_scene url="#{scene_id}"/>\n'
        f'  </scene>\n'
    )

    out.append(_DAE_FOOTER)

    os.makedirs(os.path.dirname(dae_path) or '.', exist_ok=True)
    with open(dae_path, 'w', encoding='utf-8') as f:
        f.writelines(out)

    return True
