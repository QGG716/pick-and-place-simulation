from pxr import Usd,UsdGeom
import numpy as np,json
s=Usd.Stage.Open('../seam-before-02/source-32/capture_stage.usda');cache=UsdGeom.XformCache();bb=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']);shapes=[];lights=[]
for p in s.Traverse():
 if p.GetTypeName().endswith('Light'):
  lights.append((str(p.GetPath()),np.array(cache.GetLocalToWorldTransform(p))[3,:3]))
 if (p.IsA(UsdGeom.Cube) or p.IsA(UsdGeom.Mesh)) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':
  b=bb.ComputeWorldBound(p).ComputeAlignedRange();shapes.append((p,np.array(b.GetMin()),np.array(b.GetMax())))
def ray(o,target):
 d=np.array(target)-o;hits=[]
 for p,lo,hi in shapes:
  with np.errstate(divide='ignore',invalid='ignore'): a=(lo-o)/d;b=(hi-o)/d
  start=np.max(np.minimum(a,b));end=np.min(np.maximum(a,b))
  if not (end>=max(start,0) and start<.99999 and end>1e-5):continue
  mat=np.array(cache.GetLocalToWorldTransform(p));inv=np.linalg.inv(mat);oo=(np.r_[o,1]@inv)[:3];dd=(np.r_[d,0]@inv)[:3]
  if p.IsA(UsdGeom.Cube):
   size=UsdGeom.Cube(p).GetSizeAttr().Get()/2
   with np.errstate(divide='ignore',invalid='ignore'): a=(-size-oo)/dd;b=(size-oo)/dd
   t=np.max(np.minimum(a,b));e=np.min(np.maximum(a,b))
   if e>=max(t,0) and t<.99999 and e>1e-5:hits.append((max(t,0),str(p.GetPath())))
  else:
   mesh=UsdGeom.Mesh(p);v=np.array(mesh.GetPointsAttr().Get());idx=np.array(mesh.GetFaceVertexIndicesAttr().Get());cnt=np.array(mesh.GetFaceVertexCountsAttr().Get());tri=[];off=0
   for n in cnt:
    tri.extend([[idx[off],idx[off+k],idx[off+k+1]] for k in range(1,int(n)-1)]);off+=n
   pts=v[np.array(tri)];e1=pts[:,1]-pts[:,0];e2=pts[:,2]-pts[:,0];h=np.cross(dd,e2);det=(e1*h).sum(1)
   with np.errstate(divide='ignore',invalid='ignore'):
    f=1/det;ss=oo-pts[:,0];u=f*(ss*h).sum(1);q=np.cross(ss,e1);vv=f*(q*dd).sum(1);tt=f*(e2*q).sum(1)
   ok=(abs(det)>1e-10)&(u>=0)&(vv>=0)&(u+vv<=1)&(tt>1e-5)&(tt<.99999)
   if ok.any():hits.append((float(tt[ok].min()),str(p.GetPath())))
 return sorted(hits)
rows=[]
for point in [[.7438815,-.64493,2.7],[-.095007,-.408082,2.7],[3.343856,-1.15,2.618795]]:
 for name,o in lights:
  if 'Distant' in name:continue
  hits=ray(o,point);rows.append({'point':point,'light':name,'origin':o.tolist(),'hits':hits});print(point,name.split('/')[-2:],hits[:2],flush=True)
open('../seam-light-rays.json','w').write(json.dumps(rows,indent=2))
