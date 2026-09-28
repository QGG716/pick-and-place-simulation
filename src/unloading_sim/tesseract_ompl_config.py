"""Explicit native planner selection; no implicit planner or strategy fallback."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class OMPLPlannerConfig:
    name: str = "rrt_connect"
    max_samples: int = 10000
    max_roadmap_vertices: int = 10002
    max_roadmap_edges: int = 50010

    def __post_init__(self):
        if self.name not in ("rrt_connect", "lazy_prm"):
            raise ValueError(f"unknown OMPL planner: {self.name}")
        for name, minimum, maximum in (("max_samples", 1, 1000000),
                ("max_roadmap_vertices", 2, 1000002),
                ("max_roadmap_edges", 6, 5000010)):
            value = getattr(self, name)
            if type(value) is not int or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")

    def to_mapping(self):
        return asdict(self)
