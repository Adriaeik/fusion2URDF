"""Pure-Python deserialization for ``debug/snapshot.json``.

The exporter already serializes dataclasses generically.  This module provides
the inverse operation so model-building and frame tests can run from cached
Fusion extraction data, including authoritative ``transform2`` rotations.
"""

from __future__ import annotations

import json
from dataclasses import fields
from typing import Optional

from .data_types import (
    FusionBodyInfo,
    FusionJoint,
    FusionOccurrence,
    FusionSnapshot,
    InertiaTensor,
    RigidGroupInfo,
    Transform3D,
)


def load_snapshot(path: str) -> FusionSnapshot:
    with open(path, "r", encoding="utf-8") as handle:
        return snapshot_from_dict(json.load(handle))


def snapshot_from_dict(data: dict) -> FusionSnapshot:
    item = dict(data)
    occurrences = item.pop("occurrences", {}) or {}
    joints = item.pop("joints", {}) or {}
    rigid_groups = item.pop("rigid_groups", []) or []
    snapshot = FusionSnapshot(**_filtered_kwargs(FusionSnapshot, item))
    snapshot.occurrences = {
        path: _occurrence_from_dict(value)
        for path, value in occurrences.items()
    }
    snapshot.joints = {
        name: FusionJoint(**_filtered_kwargs(FusionJoint, value))
        for name, value in joints.items()
    }
    snapshot.rigid_groups = [
        RigidGroupInfo(**_filtered_kwargs(RigidGroupInfo, value))
        for value in rigid_groups
    ]
    return snapshot


def _occurrence_from_dict(data: dict) -> FusionOccurrence:
    item = dict(data)
    local_transform = item.pop("local_transform", None)
    transform2 = item.pop("transform2", None)
    global_transform = item.pop("global_transform", None)
    inertia_origin = item.pop("inertia_at_origin", None)
    inertia_com = item.pop("inertia_at_com", None)
    bodies = item.pop("bodies", []) or []

    occurrence = FusionOccurrence(**_filtered_kwargs(FusionOccurrence, item))
    occurrence.local_transform = _transform_from_dict(local_transform) or Transform3D()
    occurrence.transform2 = _transform_from_dict(transform2)
    occurrence.global_transform = _transform_from_dict(global_transform) or Transform3D()
    occurrence.inertia_at_origin = _inertia_from_dict(inertia_origin) or InertiaTensor()
    occurrence.inertia_at_com = _inertia_from_dict(inertia_com) or InertiaTensor()
    occurrence.bodies = [_body_from_dict(value) for value in bodies]
    return occurrence


def _body_from_dict(data: dict) -> FusionBodyInfo:
    item = dict(data)
    inertia = item.pop("inertia_at_origin", None)
    body = FusionBodyInfo(**_filtered_kwargs(FusionBodyInfo, item))
    body.inertia_at_origin = _inertia_from_dict(inertia)
    return body


def _transform_from_dict(data: Optional[dict]) -> Optional[Transform3D]:
    if not data:
        return None
    translation = tuple(data.get("translation", (0.0, 0.0, 0.0)))
    rotation = data.get("rotation", (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    if rotation and isinstance(rotation[0], (list, tuple)):
        rotation = tuple(value for row in rotation for value in row)
    else:
        rotation = tuple(rotation)
    return Transform3D(translation=translation, rotation=rotation)


def _inertia_from_dict(data: Optional[dict]) -> Optional[InertiaTensor]:
    if not data:
        return None
    return InertiaTensor(**_filtered_kwargs(InertiaTensor, data))


def _filtered_kwargs(cls, data: dict) -> dict:
    allowed = {field.name for field in fields(cls)}
    return {key: value for key, value in data.items() if key in allowed}
