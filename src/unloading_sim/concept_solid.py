"""Convex concept end relief, with an outward OBB for broad-phase queries."""
from dataclasses import dataclass
import numpy as np
from .geometry import OBB

# Vertex order is the same as OBB.corners(): x, y, z signs.
FACES = ((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))


@dataclass(frozen=True)
class ConceptSolid(OBB):
    solid: dict = None

    def __post_init__(self):
        super().__post_init__()
        v = np.asarray(self.solid["vertices_m"], float)
        if (v.shape != (8,3) or not np.isfinite(v).all() or
                np.any(np.abs(v)>self.half_extents+1e-12) or
                self.solid["faces"] != [list(f) for f in FACES]):
            raise ValueError("INVALID_CONCEPT_SOLID")


def relieved_transverse(box, slope):
    """Full top footprint; fixed underside bevel clears the inclined end.

    At depth d the seam face retreats slope*d in +X. This is actual displayed
    and native collision geometry, not a collision-only shrink.
    """
    if not np.isfinite(slope) or slope<=0 or 2*box.half_extents[2]*slope>=2*box.half_extents[0]:
        raise ValueError("INVALID_SEAM_RELIEF")
    vertices = OBB(np.zeros(3), box.half_extents, np.eye(3)).corners()
    for v in vertices:
        if v[0]<0:
            v[0] += slope*(box.half_extents[2]-v[2])
    return ConceptSolid(box.center, box.half_extents, box.rotation, box.name, box.category,
        dict(kind="convex_concept_solid_v1", vertices_m=vertices.tolist(),
             faces=[list(f) for f in FACES], purpose="FULL_TOP_FOOTPRINT_UNDERSIDE_SEAM_RELIEF"))


def solid_vertices(box):
    if getattr(box, "solid", None):
        return np.asarray(box.solid["vertices_m"])@box.rotation.T+box.center
    return box.corners()


def solid_signed_gap(a, b):
    """Convex-polytope SAT; separation bound, never penetration depth."""
    va, vb = solid_vertices(a), solid_vertices(b)
    edges, axes = [], []
    for v in (va,vb):
        es = []
        for f in FACES:
            axes.append(np.cross(v[f[1]]-v[f[0]],v[f[2]]-v[f[1]]))
            es.extend(v[f[(i+1)%4]]-v[f[i]] for i in range(4))
        edges.append(es)
    axes.extend(np.cross(x,y) for x in edges[0] for y in edges[1])
    result = -np.inf
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm<=1e-12:continue
        x,y=va@(axis/norm),vb@(axis/norm)
        result=max(result,float(max(y.min()-x.max(),x.min()-y.max())))
    if not np.isfinite(result):raise ValueError("UNKNOWN_SOLID_GAP")
    return result
