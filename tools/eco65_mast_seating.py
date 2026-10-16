"""Vertical seating of original triangle surfaces, with fixed XY and rotation."""
import numpy as np


def _cross(a,b):
    return a[0]*b[1]-a[1]*b[0]


def _intersection(a,b):
    """Convex XY intersection; extrema of both triangle planes occur at its vertices."""
    if _cross(b[1]-b[0],b[2]-b[0])<0:b=b[::-1]
    poly=list(a)
    for start,end in zip(b,np.roll(b,-1,axis=0)):
        if not poly:break
        edge=end-start;result=[];previous=poly[-1];old=_cross(edge,previous-start)
        for point in poly:
            new=_cross(edge,point-start)
            if (new>=0)!=(old>=0):result.append(previous+(point-previous)*old/(old-new))
            if new>=0:result.append(point)
            previous=point;old=new
        poly=result
    return np.asarray(poly)


def vertical_seating(lower_triangles,upper_triangles):
    """Smallest upper-body Z translation ensuring vertical envelope separation.

    No scale, XY shift, rotation, mesh edits or collision exemption. Uses the
    actual projected triangle overlap, not global bounding-box heights. This
    establishes contact/separation for these surfaces only, not bolted mounting.
    """
    def planes(triangles):
        t=np.asarray(triangles,float)
        if t.ndim!=3 or t.shape[1:]!=(3,3) or not np.isfinite(t).all():raise ValueError('INVALID_TRIANGLES')
        n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);ids=np.flatnonzero(np.abs(n[:,2])>1e-16)
        t=t[ids];n=n[ids]
        coeff=np.column_stack([-n[:,0]/n[:,2],-n[:,1]/n[:,2],np.einsum('ij,ij->i',n,t[:,0])/n[:,2]])
        return t,coeff,ids
    lower,lc,li=planes(lower_triangles);upper,uc,ui=planes(upper_triangles)
    lo,hi=lower[:,:,:2].min(1),lower[:,:,:2].max(1);best=-np.inf;witness=None;pairs=0
    for j,triangle in enumerate(upper):
        xy=triangle[:,:2];indices=np.flatnonzero(np.all((xy.max(0)>=lo)&(hi>=xy.min(0)),axis=1))
        for i in indices:
            polygon=_intersection(xy,lower[i,:,:2])
            if not len(polygon):continue
            pairs+=1;gap=polygon@(lc[i,:2]-uc[j,:2])+lc[i,2]-uc[j,2];k=int(np.argmax(gap))
            if gap[k]>best:
                best=float(gap[k]);p=polygon[k];z=float(p@lc[i,:2]+lc[i,2])
                witness=dict(lower_triangle=int(li[i]),upper_triangle=int(ui[j]),contact_m=[float(p[0]),float(p[1]),z])
    if witness is None:raise ValueError('NO_PROJECTED_MOUNTING_OVERLAP')
    return dict(upper_z_translation_m=best,witness=witness,overlapping_triangle_pairs=pairs,
                method='Exact linear height difference on projected triangle intersections',
                scope='ORIGINAL_MESH_VERTICAL_ENVELOPE_CONTACT_ONLY_NOT_HARDWARE_MOUNTING')


def conformal_seat(lower_triangles,upper_triangles,max_thickness_m=None):
    """Closed convex cells filling verified gaps between the two surface meshes.

    Each cell follows original triangle planes. Source holes and footprint edges
    remain open. Unsupported or ambiguous projected regions are omitted rather
    than filled through original solids. Returned coordinates share both inputs'
    frame; this is a new project-derived mounting part, never original CAD.
    """
    def prepare(t,sign):
        t=np.asarray(t,float);n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);t=t[n[:,2]*sign>1e-16]
        e=t[:,1]-t[:,0];f=t[:,2]-t[:,0];det=e[:,0]*f[:,1]-e[:,1]*f[:,0]
        return t,e,f,det
    lower=prepare(lower_triangles,1);upper=prepare(upper_triangles,-1)
    def heights(surface,points,maximum):
        t,e,f,det=surface;d=points[:,None,:]-t[None,:,0,:2]
        a=(d[:,:,0]*f[:,1]-d[:,:,1]*f[:,0])/det;b=(e[:,0]*d[:,:,1]-e[:,1]*d[:,:,0])/det
        valid=(a>=-1e-9)&(b>=-1e-9)&(a+b<=1+1e-9)
        z=t[:,0,2]+a*e[:,2]+b*f[:,2]
        return np.max(np.where(valid,z,-np.inf),axis=1) if maximum else np.min(np.where(valid,z,np.inf),axis=1)
    def z_on(t,points):
        n=np.cross(t[1]-t[0],t[2]-t[0]);return (n@t[0]-points@n[:2])/n[2]
    lo=lower[0][:,:,:2].min(1);hi=lower[0][:,:,:2].max(1)
    vertices=[];faces=[];count=0;area=0.;rejected_area=0.;max_error=0.
    for top in upper[0]:
        xy=top[:,:2];indices=np.flatnonzero(np.all((xy.max(0)>=lo)&(hi>=xy.min(0)),axis=1))
        for i in indices:
            bottom=lower[0][i];p=_intersection(xy,bottom[:,:2])
            if len(p)<3:continue
            signed=sum(_cross(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))/2
            if abs(signed)<1e-14:continue
            if signed<0:p=p[::-1]
            # Check vertices and centroid against the outermost original surfaces.
            sample=np.concatenate([p,p.mean(0)[None,:]])
            bz=z_on(bottom,sample);tz=z_on(top,sample)
            error=max(float(np.max(np.abs(bz-heights(lower,sample,True)))),float(np.max(np.abs(tz-heights(upper,sample,False)))))
            if error>1e-7 or np.min(tz-bz)<-1e-8 or (max_thickness_m is not None and np.max(tz-bz)>max_thickness_m):
                rejected_area+=abs(signed);continue
            if np.max(tz-bz)<1e-7:continue
            max_error=max(max_error,error);n=len(p);offset=len(vertices)
            vertices.extend(np.column_stack([p,bz[:n]]));vertices.extend(np.column_stack([p,tz[:n]]))
            for k in range(1,n-1):faces.extend([[offset,offset+k+1,offset+k],[offset+n,offset+n+k,offset+n+k+1]])
            for k in range(n):
                j=(k+1)%n;faces.extend([[offset+k,offset+j,offset+n+j],[offset+k,offset+n+j,offset+n+k]])
            count+=1;area+=abs(signed)
    if not count:raise ValueError('NO_VERIFIED_SEAT_PATCHES')
    return np.array(vertices),np.array(faces),dict(cells=count,contact_area_m2=area,
        omitted_projected_area_m2=rejected_area,max_surface_error_m=max_error,
        max_thickness_m=max_thickness_m,
        construction='Closed convex cells bounded by original surface triangles; exact triangle collision mesh',
        provenance='PROJECT_DERIVED_CONFORMAL_J1_SEAT_NOT_ORIGINAL_CAD')
