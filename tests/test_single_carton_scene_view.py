"""Read-only 3D environment view (user request 2026-11-27): geometry helpers and a headless render."""
import os
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.scene_view import box_solid, hull_solid


def test_box_faces_point_outward_and_hull_falls_back_to_a_box():
    P=np.eye(4);P[:3,3]=[1,2,3];v,faces=box_solid(P,[.2,.4,.6])
    np.testing.assert_allclose(v.min(0),[.9,1.8,2.7]);np.testing.assert_allclose(v.max(0),[1.1,2.2,3.3])
    centre=v.mean(0)
    for f in faces:
        a,b,c=v[f[0]],v[f[1]],v[f[2]];n=np.cross(b-a,c-a)
        assert n@(v[f].mean(0)-centre)>0
    flat=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0]],float)   # degenerate: no 3D hull
    points,faces,normals=hull_solid(flat)
    assert len(faces)==6 and normals is None
    cube=np.array([[x,y,z] for x in (0,1) for y in (0,1) for z in (0,1)],float)
    points,faces,normals=hull_solid(cube)
    assert len(faces)==12 and np.all(np.abs(np.linalg.norm(normals,axis=1)-1)<1e-9)


@pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='needs a display (xvfb-run)')
def test_view_renders_layout_task_scene_and_toggles_wireframe():
    import tkinter as tk
    from unloading_sim.workbench.single_carton.scene_view import SceneView3D
    root=tk.Tk()
    try:
        P=np.eye(4);P[:3,3]=[-.7,0,.3]
        manager=SimpleNamespace(scene=dict(world=[dict(id='rgbd_x',category='observed',pose=P.tolist(),size=[.04]*3)]),
                                boxes=[dict(id='b0',detection_index=0,pose=P.tolist(),size_m=[.2,.15,.1])],observation=None)
        v=SceneView3D(root,config={},planner=SimpleNamespace(ready=False),model=None,manager=manager,backend=SimpleNamespace(joint=None))
        v.pack();v.layout=dict(kept=[dict(id='site_layout_layout_part_466',pose=np.eye(4).tolist(),size=[.01,1,1],name='panel')],
                               pruned=[],bounds=[[-1,-1,-.2],[1,1,1]],source_sha256='a4b6ed1298f7491b'+'0'*48)
        root.update();v.render();root.update()
        assert v.canvas.find_all() and any(k=='site_layout_layout_part_466' for k,_ in v._names.values())
        before=len(v.canvas.find_withtag('all'))
        v.wire={'site_layout_layout_part_466'};v.render()
        assert len(v.canvas.find_withtag('all'))!=before
        v.offset.set('-500');items=v.items()
        panel=next(i for i in items if i.get('key')=='site_layout_layout_part_466')
        assert panel['pose'][1][3]==pytest.approx(-.5)
        v.layout=dict(v.layout,source_sha256='0b5be708'+'0'*56)      # later revision: ids renumbered, no offset
        panel=next(i for i in v.items() if i.get('key')=='site_layout_layout_part_466')
        assert panel['pose'][1][3]==pytest.approx(0) and panel['label'] is None
    finally:
        root.destroy()
