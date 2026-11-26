"""Remove RGB-D returns inside source-bound robot solids at q_capture only.

This does not alter any planning collision pair. Nonconvex input is retained as
an environmental return rather than silently replacing it by a convex hull.
"""
from urllib.parse import urlparse,unquote
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh
from .geometry import transform


def origin(element):
    T=np.eye(4)
    if element is not None:
        T[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return T


class RobotSelfFilter:
    def __init__(self,urdf,model):
        self.model=model;u=ET.fromstring(urdf);self.solids=[];cache={}
        self.fixed={j.find('child').get('link'):(j.find('parent').get('link'),origin(j.find('origin')))
                    for j in u.findall('joint') if j.get('type')=='fixed'}
        for link in u.findall('link'):
            for c in link.findall('collision'):
                geometry=c.find('geometry');box=geometry.find('box');mesh=geometry.find('mesh')
                if box is not None:
                    half=np.fromstring(box.get('size'),sep=' ')/2
                    equations=np.c_[np.r_[np.eye(3),-np.eye(3)],-np.r_[half,half]];bounds=np.array([-half,half])
                elif mesh is not None:
                    key=(mesh.get('filename'),mesh.get('scale','1 1 1'))
                    if key not in cache:
                        path=unquote(urlparse(key[0]).path)
                        solid=trimesh.load(path,force='mesh',process=False)
                        if not solid.is_convex:cache[key]=None
                        else:
                            vertices=np.asarray(solid.vertices)*np.fromstring(key[1],sep=' ')
                            cache[key]=(ConvexHull(vertices).equations,np.array([vertices.min(0),vertices.max(0)]))
                    if cache[key] is None:continue
                    equations,bounds=cache[key]
                else:continue
                self.solids.append((link.get('name'),origin(c.find('origin')),equations,bounds))
    def world_solids(self,q):
        """The same proven convex solids, for exact shadow subtraction only."""
        frames={name:self.model.world_from_base@T for name,T in self.model.frames(q).items()}
        frames['world']=self.model.world_from_base
        def frame(name):
            if name not in frames:
                parent,T=self.fixed[name];frames[name]=frame(parent)@T
            return frames[name]
        result=[]
        for name,T,eq,bounds in self.solids:
            W=frame(name)@T;normals=eq[:,:3]@W[:3,:3].T
            center=W[:3,:3]@bounds.mean(0)+W[:3,3];half=abs(W[:3,:3])@((bounds[1]-bounds[0])/2)
            result.append(dict(equations=np.c_[normals,eq[:,3]-normals@W[:3,3]],lower=center-half,upper=center+half))
        return result
    def contains(self,points,q,group_size=1):
        """Per group: all group_size points inside one robot solid at q.

        Speed (2026-11-26, "构建当前帧同构仿真环境要提速"): groups are first
        reduced to those inside the (slightly inflated) AABB of all solids,
        then of each link's solids. These are necessary conditions of the
        unchanged per-solid AABB and half-space test, so the result is
        identical to testing every point against every solid.
        """
        points=np.asarray(points);inside=np.zeros(len(points)//group_size,bool)
        if len(points)%group_size:raise ValueError('SELF_FILTER_GROUP_SHAPE')
        if not len(points) or not self.solids:return inside
        frames={name:self.model.world_from_base@T for name,T in self.model.frames(q).items()}
        frames['world']=self.model.world_from_base
        def frame(name):
            if name not in frames:
                parent,T=self.fixed[name];frames[name]=frame(parent)@T
            return frames[name]
        solids=[]
        for name,T,equations,bounds in self.solids:
            world=frame(name)@T
            center=world[:3,:3]@bounds.mean(0)+world[:3,3];half=abs(world[:3,:3])@((bounds[1]-bounds[0])/2)
            solids.append((name,world,equations,center,half))
        # Prefilter boxes are inflated so float rounding can only keep more.
        pad=1e-6
        def within(P,lo,hi):return np.all((P>=lo-pad)&(P<=hi+pad),axis=1)
        lo=np.min([c-h for _,_,_,c,h in solids],0);hi=np.max([c+h for _,_,_,c,h in solids],0)
        groups=np.flatnonzero(within(points,lo,hi).reshape(-1,group_size).all(1))
        if not len(groups):return inside
        P=points[(groups[:,None]*group_size+np.arange(group_size)).ravel()]
        sub=np.zeros(len(groups),bool)
        # Consecutive solids of one link share one prefilter (order unchanged).
        runs=[];start=0
        for i in range(1,len(solids)+1):
            if i==len(solids) or solids[i][0]!=solids[start][0]:runs.append((start,i));start=i
        for a,b in runs:
            link_lo=np.min([s[3]-s[4] for s in solids[a:b]],0);link_hi=np.max([s[3]+s[4] for s in solids[a:b]],0)
            near=np.flatnonzero(~sub & within(P,link_lo,link_hi).reshape(-1,group_size).all(1))
            if not len(near):continue
            for _,world,equations,center,half in solids[a:b]:
                near=near[~sub[near]]
                if not len(near):break
                Q=P.reshape(len(groups),group_size,3)[near].reshape(-1,3)
                box=np.all(abs(Q-center)<=half,axis=1)
                candidates=np.flatnonzero(box)
                if not len(candidates):continue
                local=transform(np.linalg.inv(world),Q[candidates])
                same_solid=np.zeros(len(Q),bool)
                same_solid[candidates]=np.all(local@equations[:,:3].T+equations[:,3]<=1e-9,axis=1)
                sub[near[same_solid.reshape(-1,group_size).all(1)]]=True
        inside[groups]=sub
        return inside
