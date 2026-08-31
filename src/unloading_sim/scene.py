"""Trailer, carton, and conveyor scene construction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from .geometry import OBB, rotation_matrix_from_rpy


@dataclass
class TrailerScene:
    obstacles: list[OBB]
    cartons: list[OBB]
    metadata: dict[str, Any]

    @property
    def all_obstacles(self) -> list[OBB]:
        return self.obstacles + self.cartons

    def carton(self, name: str) -> OBB:
        for carton in self.cartons:
            if carton.name == name:
                return carton
        raise KeyError(f"Unknown carton: {name}")

    def obstacles_without(self, names: Iterable[str]) -> list[OBB]:
        ignored = set(names)
        return [o for o in self.all_obstacles if o.name not in ignored]


def _obb_from_dict(data: dict[str, Any], category: str) -> OBB:
    center = np.asarray(data["center"], dtype=float)
    size = np.asarray(data["size"], dtype=float)
    rpy = np.asarray(data.get("rpy", [0.0, 0.0, 0.0]), dtype=float)
    return OBB(
        center=center,
        half_extents=size / 2.0,
        rotation=rotation_matrix_from_rpy(*rpy),
        name=str(data["name"]),
        category=category,
    )


def build_trailer_walls(
    length: float,
    width: float,
    height: float,
    wall_thickness: float,
    floor_thickness: float,
) -> list[OBB]:
    """Build an open-at-x=0 trailer with walls extending along +x."""
    ident = np.eye(3)
    t = wall_thickness
    ft = floor_thickness
    return [
        OBB(
            center=np.array([length / 2.0, 0.0, -ft / 2.0]),
            half_extents=np.array([length / 2.0, width / 2.0, ft / 2.0]),
            rotation=ident,
            name="trailer_floor",
            category="trailer",
        ),
        OBB(
            center=np.array([length / 2.0, width / 2.0 + t / 2.0, height / 2.0]),
            half_extents=np.array([length / 2.0, t / 2.0, height / 2.0]),
            rotation=ident,
            name="trailer_left_wall",
            category="trailer",
        ),
        OBB(
            center=np.array([length / 2.0, -width / 2.0 - t / 2.0, height / 2.0]),
            half_extents=np.array([length / 2.0, t / 2.0, height / 2.0]),
            rotation=ident,
            name="trailer_right_wall",
            category="trailer",
        ),
        OBB(
            center=np.array([length / 2.0, 0.0, height + t / 2.0]),
            half_extents=np.array([length / 2.0, width / 2.0, t / 2.0]),
            rotation=ident,
            name="trailer_roof",
            category="trailer",
        ),
        OBB(
            center=np.array([length + t / 2.0, 0.0, height / 2.0]),
            half_extents=np.array([t / 2.0, width / 2.0, height / 2.0]),
            rotation=ident,
            name="trailer_front_wall",
            category="trailer",
        ),
    ]


def _merge_config(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_config(merged[key], value)
        else:
            merged[key] = value
    return merged


def _load_config_dict(path: Path, ancestors: tuple[Path, ...] = ()) -> dict[str, Any]:
    resolved = path.resolve()
    if resolved in ancestors:
        chain = " -> ".join(str(item) for item in (*ancestors, resolved))
        raise ValueError(f"cyclic config inheritance: {chain}")
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    parent_config = cfg.pop("extends", None)
    if parent_config is None:
        return cfg
    parent_path = path.parent / str(parent_config)
    parent_cfg = _load_config_dict(parent_path, (*ancestors, resolved))
    return _merge_config(parent_cfg, cfg)


def load_scene_config(path: str | Path) -> tuple[TrailerScene, dict[str, Any]]:
    path = Path(path)
    # Plans are portable artifacts and may have been generated on Windows but
    # certified on Linux. Resolve the alternate separator only when the path
    # does not exist, so literal backslashes remain valid on POSIX filesystems.
    if not path.exists() and "\\" in str(path):
        portable_path = Path(str(path).replace("\\", "/"))
        if portable_path.exists():
            path = portable_path
    cfg = _load_config_dict(path)

    trailer = cfg["scene"]["trailer"]
    obstacles = build_trailer_walls(
        length=float(trailer["length"]),
        width=float(trailer["width"]),
        height=float(trailer["height"]),
        wall_thickness=float(trailer.get("wall_thickness", 0.05)),
        floor_thickness=float(trailer.get("floor_thickness", 0.08)),
    )

    for item in cfg["scene"].get("static_obstacles", []):
        obstacles.append(_obb_from_dict(item, category="static"))

    cartons = [_obb_from_dict(item, category="carton") for item in cfg["scene"].get("cartons", [])]
    scene = TrailerScene(
        obstacles=obstacles,
        cartons=cartons,
        metadata={
            "source": str(path),
            "trailer_interior": {
                "length": float(trailer["length"]),
                "width": float(trailer["width"]),
                "height": float(trailer["height"]),
            },
        },
    )
    return scene, cfg
