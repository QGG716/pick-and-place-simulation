"""Role-based edits for the hash-pinned A06 connected frame components."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def support_edits(mesh, vertices, belt_top, scale, clearance):
    counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    _,weld=np.unique(np.round(vertices,8),axis=0,return_inverse=True)
    starts=np.r_[0,np.cumsum(counts)[:-1]]
    following=np.arange(len(indices))+1;following[np.cumsum(counts)-1]=starts
    a=weld[indices];b=weld[indices[following]]
    graph=coo_matrix((np.ones(len(a)),(a,b)),shape=(weld.max()+1,)*2).tocsr()
    _,labels=connected_components(graph,directed=False)
    face_labels=labels[a[starts]];vertex_labels=labels[weld]
    target=belt_top-clearance/scale
    offsets=np.zeros(len(counts));upper=np.full(len(counts),np.inf);audit=[];roles=[]
    for label in np.unique(face_labels):
        v=vertices[vertex_labels==label];lo=v.min(0);hi=v.max(0);size=hi-lo
        if hi[2]<=target:continue
        if size[0]>1.9 and .19<size[2]<.21:role='side_rail';shift=target-hi[2];cut=None
        elif size[0]<.006 and .18<size[2]<.20:role='rail_end_plate';shift=target-1.8028864288;cut=None
        elif size[2]>2.2 and size[0]<.09:role='upright';shift=0.;cut=target-.0250001526
        elif .024<size[2]<.026 and lo[2]>2.28:role='upright_cap';shift=target-hi[2];cut=None
        else:raise ValueError('UNCLASSIFIED_A06_HIGH_COMPONENT:'+str([lo.tolist(),hi.tolist()]))
        mask=face_labels==label;offsets[mask]=shift*scale
        if cut is not None:upper[mask]=(cut-belt_top)*scale
        roles.append(role);audit.append(dict(role=role,source_bounds_m=[lo.tolist(),hi.tolist()],source_z_shift_m=shift,source_upper_cut_m=cut))
    expected={'side_rail':2,'rail_end_plate':4,'upright':2,'upright_cap':2}
    if {r:roles.count(r) for r in set(roles)}!=expected:raise ValueError('A06_ROLE_COUNT_CHANGED:'+str({r:roles.count(r) for r in set(roles)}))
    return offsets,upper,audit
