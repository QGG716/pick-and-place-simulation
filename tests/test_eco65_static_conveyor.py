"""Focused USD clip test: shortened supports retain UVs and face ownership."""
import sys
import unittest
from pathlib import Path
import numpy as np
try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError:
    raise unittest.SkipTest('Optional USD runtime is required for the static support clip test')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from eco65_official_conveyor import clipped_mesh


class SupportClipTest(unittest.TestCase):
    def test_crossing_face_interpolates_uv_and_removes_lower_face(self):
        stage=Usd.Stage.CreateInMemory();mesh=UsdGeom.Mesh.Define(stage,'/Support')
        mesh.CreatePointsAttr([(-1,0,-1),(1,0,-1),(1,0,1),(-1,0,1),(0,0,-2)])
        mesh.CreateFaceVertexCountsAttr([4,3]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3,0,4,1])
        mesh.CreateNormalsAttr([Gf.Vec3f(0,-1,0)]*7);mesh.SetNormalsInterpolation('faceVarying')
        uv=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying')
        uv.Set([(0,0),(1,0),(1,1),(0,1),(0,0),(.5,-.5),(1,0)])
        vertices,counts,attrs,mapping=clipped_mesh(mesh,np.eye(4),0.)
        self.assertEqual(counts,[4]);self.assertEqual(mapping,{0:0})
        self.assertTrue(np.allclose(vertices.min(0),[-1,0,0]))
        self.assertTrue(np.allclose(vertices.max(0),[1,0,1]))
        self.assertTrue(np.allclose(np.asarray(attrs['st'])[vertices[:,2]==0,1],.5))
        self.assertTrue(np.allclose(attrs['_normal'],[0,-1,0]))

    def test_role_cut_leaves_unselected_face_and_interpolates_uv(self):
        stage=Usd.Stage.CreateInMemory();mesh=UsdGeom.Mesh.Define(stage,'/Support')
        mesh.CreatePointsAttr([(0,0,0),(1,0,0),(1,0,1),(0,0,1)])
        mesh.CreateFaceVertexCountsAttr([4,4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]*2)
        mesh.CreateNormalsAttr([Gf.Vec3f(0,-1,0)]*8);mesh.SetNormalsInterpolation('faceVarying')
        uv=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying')
        uv.Set([(0,0),(1,0),(1,1),(0,1)]*2)
        vertices,counts,attrs,mapping=clipped_mesh(mesh,np.eye(4),0.,(np.zeros(2),np.array([.6,np.inf])))
        self.assertEqual(counts,[4,4]);self.assertEqual(mapping,{0:0,1:1})
        self.assertAlmostEqual(vertices[:4,2].max(),.6)
        self.assertAlmostEqual(vertices[4:,2].max(),1.)
        self.assertTrue(np.allclose(np.asarray(attrs['st'])[:4][vertices[:4,2]==.6,1],.6))


if __name__=='__main__':unittest.main()
