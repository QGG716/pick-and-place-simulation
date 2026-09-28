"""Explicit native planner selection; no implicit planner or strategy fallback."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class EndpointSamplingConfig:
    type: str = "endpoint_mixture"
    local_probability: float = .5
    joint_span_half_width_fraction: float = .05

    def __post_init__(self):
        if self.type != "endpoint_mixture":
            raise ValueError("unknown sampling strategy")
        for name, upper in (("local_probability", 1.), ("joint_span_half_width_fraction", .25)):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or not 0 < value < upper):
                raise ValueError(f"{name} must be finite and in (0, {upper})")


@dataclass(frozen=True)
class OMPLPlannerConfig:
    name: str = "rrt_connect"
    max_samples: int = 10000
    max_roadmap_vertices: int = 10002
    max_roadmap_edges: int = 50010
    sampling: EndpointSamplingConfig | None = None

    def __post_init__(self):
        if self.name not in ("rrt_connect", "lazy_prm"):
            raise ValueError(f"unknown OMPL planner: {self.name}")
        if isinstance(self.sampling, dict):
            try:
                object.__setattr__(self, "sampling", EndpointSamplingConfig(**self.sampling))
            except TypeError as exc:
                raise ValueError("unsupported sampling fields") from exc
        if self.sampling is not None and (not isinstance(self.sampling, EndpointSamplingConfig) or self.name != "lazy_prm"):
            raise ValueError("endpoint sampling requires lazy_prm and EndpointSamplingConfig")
        for name, minimum, maximum in (("max_samples", 1, 1000000),
                ("max_roadmap_vertices", 2, 1000002),
                ("max_roadmap_edges", 6, 5000010)):
            value = getattr(self, name)
            if type(value) is not int or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")

    def to_mapping(self):
        result = asdict(self)
        if self.sampling is None:
            result.pop("sampling")  # Preserve the exact preceding default wire contract.
        return result
