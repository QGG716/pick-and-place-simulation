"""Explicit decimation grid. Depth is sampled, never averaged across an edge."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class WorkingGrid:
    height: int
    width: int
    divisor: int = 1

    def __post_init__(self):
        if self.divisor not in (1,2) or min(self.height,self.width)<1 or self.height%self.divisor or self.width%self.divisor:
            raise ValueError('INVALID_WORKING_RESOLUTION')

    def sample(self,array):
        a=np.asarray(array)
        if a.shape[:2]!=(self.height,self.width):raise ValueError('WORKING_IMAGE_SHAPE_MISMATCH')
        return a[::self.divisor,::self.divisor].copy()

    def intrinsics(self,K):
        K=np.asarray(K,float).reshape(3,3).copy()
        if not np.isfinite(K).all() or min(K[0,0],K[1,1])<=0 or not np.array_equal(K[2],[0,0,1]):raise ValueError('INVALID_K')
        K[:2]/=self.divisor;return K

    def boxes(self,boxes):
        b=np.asarray(boxes,float)
        if b.shape==(0,):return b.reshape(0,4)
        if b.ndim!=2 or b.shape[1]!=4 or not np.isfinite(b).all() or np.any(b[:,2:]<=b[:,:2]) or np.any(b<0) or np.any(b[:,[0,2]]>self.width) or np.any(b[:,[1,3]]>self.height):
            raise ValueError('INVALID_XYXY_EDGE_BOX')
        return b/self.divisor

    def restore(self,array):
        a=np.asarray(array)
        if a.shape[-2:]!=(self.height//self.divisor,self.width//self.divisor):raise ValueError('WORKING_MASK_SHAPE_MISMATCH')
        return np.repeat(np.repeat(a,self.divisor,axis=-2),self.divisor,axis=-1)

    def to_dict(self):
        return dict(source_resolution=[self.width,self.height],working_resolution=[self.width//self.divisor,self.height//self.divisor],
            divisor=self.divisor,pixel_centers='integer centers; work(u,v) reads original(divisor*u,divisor*v)',
            bbox='xyxy half-open pixel-index bounds / divisor; upper image edge allowed',
            mask_restore='nearest cell replication; conservative native-depth verification remains required',
            depth_resampling='NONE: exact strided samples, no interpolated depth',original_to_working=[[1/self.divisor,0,0],[0,1/self.divisor,0],[0,0,1]])
