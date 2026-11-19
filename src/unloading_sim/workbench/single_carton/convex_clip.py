"""Exact convex half-space subtraction for conservative obstacle ownership.

Keep every part outside the owned solid. Dropping a whole intersecting voxel
would silently erase unowned obstacle volume at the target or robot boundary.
"""
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull,QhullError


def box_solid(pose,size):
    T=np.asarray(pose,float);half=np.asarray(size,float)/2
    normals=np.r_[T[:3,:3].T,-T[:3,:3].T]
    eq=np.c_[normals,-np.r_[half,half]-normals@T[:3,3]]
    vertices=np.array(list(product((-1.,1.),repeat=3)))*half@T[:3,:3].T+T[:3,3]
    return dict(equations=eq,lower=vertices.min(0),upper=vertices.max(0))


def clip(vertices,plane):
    v=np.asarray(vertices);d=v@plane[:3]+plane[3]
    if np.all(d<=1e-12):return v
    if np.all(d>=-1e-12):return None
    inside=v[d<=0];a,b=np.triu_indices(len(v),1);cross=d[a]*d[b]<0
    a,b=a[cross],b[cross]
    points=np.r_[inside,v[a]+(v[b]-v[a])*(d[a]/(d[a]-d[b]))[:,None]]
    points=np.unique(np.round(points,12),axis=0)
    if len(points)<4:return None
    try:return points[ConvexHull(points).vertices]
    except QhullError:return None  # zero-volume contact is not an obstacle solid


def subtract(vertices,solid):
    v=np.asarray(vertices)
    if np.any(v.max(0)<=solid['lower']) or np.any(v.min(0)>=solid['upper']):return [v]
    eq=solid['equations'];values=v@eq[:,:3].T+eq[:,3]
    if np.any(np.all(values>=0,axis=0)):return [v]
    if np.all(values<=1e-12):return []
    remaining=v;outside=[]
    for plane in eq:
        part=clip(remaining,-plane)
        if part is not None:outside.append(part)
        remaining=clip(remaining,plane)
        if remaining is None:break
    return outside


def subtract_solids(vertices,solids):
    pieces=[np.asarray(vertices)]
    for solid in solids:
        pieces=[part for v in pieces for part in subtract(v,solid)]
        if not pieces:break
    return pieces


def mesh(vertices):
    v=np.asarray(vertices);h=ConvexHull(v);tri=h.simplices.copy()
    for i,(a,b,c) in enumerate(tri):
        if np.cross(v[b]-v[a],v[c]-v[a])@h.equations[i,:3]<0:tri[i]=[a,c,b]
    return dict(vertices=v.tolist(),triangles=tri.tolist())
