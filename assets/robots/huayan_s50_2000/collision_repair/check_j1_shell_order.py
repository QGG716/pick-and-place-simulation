"""Check whether J1 closed-shell validation depends on topology insertion order."""
import json,hashlib
from inspect_missing_faces import *
from OCP.TopAbs import TopAbs_SHELL
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
def main():
 parts=read_parts();shape=next(s for n,s in parts.items() if n.startswith('J1'))
 shells=[];ex=TopExp_Explorer(shape,TopAbs_SHELL)
 while ex.More():shells.append(TopoDS.Shell_s(ex.Current()));ex.Next()
 normal=list(range(1,len(shells)));swapped=normal.copy();swapped[13],swapped[15]=swapped[15],swapped[13]
 orders={'original':normal,'reverse_inner_shells':normal[::-1],'swap_inner_14_16':swapped,'inner_16_first':[16]+[x for x in normal if x!=16]}
 checks=[]
 for name,order in orders.items():
  builder=BRepBuilderAPI_MakeSolid(shells[0])
  for i in order:builder.Add(shells[i])
  solid=builder.Solid();props=GProp_GProps();BRepGProp.VolumeProperties_s(solid,props)
  fs=faces(solid);orig=faces(shape)
  checks.append(dict(order_name=name,inner_shell_order=order,valid=bool(BRepCheck_Analyzer(solid).IsValid()),
                     signed_volume_m3=props.Mass()*1e-9,
                     all_source_faces_preserved=len(fs)==len(orig) and all(any(f.IsSame(g) for g in fs) for f in orig)))
 result=dict(schema='s50_j1_shell_order_check_v1',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
             cases=checks,no_geometry_changed=True)
 (OUT/'j1_shell_order_check.json').write_text(json.dumps(result,indent=2),encoding='utf8')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()

