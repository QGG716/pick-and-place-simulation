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
    def contains(self,points,q,group_size=1):
        points=np.asarray(points);inside=np.zeros(len(points)//group_size,bool)
        if len(points)%group_size:raise ValueError('SELF_FILTER_GROUP_SHAPE')
        frames={name:self.model.world_from_base@T for name,T in self.model.frames(q).items()}
        frames['world']=self.model.world_from_base
        def frame(name):
            if name not in frames:
                parent,T=self.fixed[name];frames[name]=frame(parent)@T
            return frames[name]
        for name,T,equations,bounds in self.solids:
            world=frame(name)@T
            center=world[:3,:3]@bounds.mean(0)+world[:3,3];half=abs(world[:3,:3])@((bounds[1]-bounds[0])/2)
            candidates=np.flatnonzero(~np.repeat(inside,group_size) & np.all(abs(points-center)<=half,axis=1))
            if len(candidates):
                local=transform(np.linalg.inv(world),points[candidates])
                same_solid=np.zeros(len(points),bool)
                same_solid[candidates]=np.all(local@equations[:,:3].T+equations[:,3]<=1e-9,axis=1)
                inside |= same_solid.reshape(-1,group_size).all(1)
        return inside
