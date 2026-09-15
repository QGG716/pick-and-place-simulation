"""Render-only, equivalent cuboid mesh and shared non-identifying carton atlas."""
from pathlib import Path
import hashlib
import math

# Outward-wound faces; parent scale remains the original collision Cube size.
FACES = (
    ((-.5,-.5,-.5),(-.5,-.5,.5),(-.5,.5,.5),(-.5,.5,-.5)),
    ((.5,-.5,-.5),(.5,.5,-.5),(.5,.5,.5),(.5,-.5,.5)),
    ((-.5,-.5,-.5),(.5,-.5,-.5),(.5,-.5,.5),(-.5,-.5,.5)),
    ((-.5,.5,-.5),(-.5,.5,.5),(.5,.5,.5),(.5,.5,-.5)),
    ((-.5,-.5,-.5),(-.5,.5,-.5),(.5,.5,-.5),(.5,-.5,-.5)),
    ((-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)),
)


def face_dimensions(size, face):
    return tuple(math.sqrt(sum(((face[j][i]-face[0][i])*size[i])**2 for i in range(3))) for j in (1,3))


def atlas_uv():
    # PNG top-to-bottom rows versus USD bottom-to-top texture coordinates.
    return [((i%3+u)/3, 1-(i//3+v)/2) for i in range(6) for u,v in ((0,0),(1,0),(1,1),(0,1))]


def create_atlas(source, output, size, tile_metres=.25):
    """Bake a material asset, never a sensor image; same design for every box."""
    import cv2
    import numpy as np
    source=Path(source); output=Path(output)
    paper=cv2.imread(str(source))
    if paper is None or paper.shape[:2]!=(512,512): raise ValueError('CARTON_TEXTURE_MISSING_OR_INVALID')
    tiles=[]
    for i,face in enumerate(FACES):
        width,height=face_dimensions(size,face)
        w,h=max(2,round(width/tile_metres*512)),max(2,round(height/tile_metres*512))
        tile=np.tile(paper,(math.ceil(h/512),math.ceil(w/512),1))[:h,:w].copy()
        if i==5:
            # 50 mm printed diffuse tape appearance; no extra surface or normals.
            a=max(0,round((width-.05)/2/width*w));b=min(w,round((width+.05)/2/width*w))
            tile[:,a:b]=(tile[:,a:b]*np.array([.72,.81,.87])).astype(np.uint8)
        elif i<4:
            # Identical small handling arrows, not a per-object code or seam cue.
            length=max(3,round(.03/height*h)); cy=round(.35*h)
            for cx in (round(.45*w),round(.55*w)):
                cv2.arrowedLine(tile,(cx,cy+length),(cx,cy),(70,85,95),max(1,round(w/width*.0015)),tipLength=.3)
        tiles.append(cv2.resize(tile,(512,512),interpolation=cv2.INTER_AREA))
    atlas=np.vstack((np.hstack(tiles[:3]),np.hstack(tiles[3:])))
    output.parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(output),atlas): raise RuntimeError('CARTON_TEXTURE_WRITE_FAILED')
    return {'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'atlas':str(output),'atlas_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
            'nominal_size_m':list(size),'paper_tile_m':tile_metres,'top_tape_width_m':.05,
            'design':'SAME_FIBRE_AND_HANDLING_ARROWS_ALL_CARTONS','normal_displacement_alpha':'NONE'}


def define_visual(stage, path):
    from pxr import Gf,Sdf,UsdGeom
    mesh=UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for face in FACES for p in face])
    mesh.CreateFaceVertexCountsAttr([4]*6);mesh.CreateFaceVertexIndicesAttr(list(range(24)))
    mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(False)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set([Gf.Vec2f(*v) for v in atlas_uv()])
    return mesh


def textured_material(stage,path,atlas):
    from pxr import Gf,Sdf,UsdShade
    if not Path(atlas).is_file(): raise ValueError('CARTON_ATLAS_MISSING')
    mat=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.9)
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(1.)
    shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
    reader=UsdShade.Shader.Define(stage,path+'/UV');reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    texture=UsdShade.Shader.Define(stage,path+'/Texture');texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(Path(atlas).resolve()))
    texture.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.78,.60,.38,1))
    texture.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('clamp')
    texture.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('clamp')
    texture.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(),'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    if not shader.GetInput('diffuseColor').HasConnectedSource(): raise RuntimeError('CARTON_TEXTURE_NOT_CONNECTED')
    return mat
