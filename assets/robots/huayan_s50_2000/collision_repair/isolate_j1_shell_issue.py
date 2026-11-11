"""Locate invalid shell addition without deleting source collision surfaces."""
import json
from inspect_missing_faces import *
from OCP.TopAbs import TopAbs_SHELL
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid

def main():
 parts=read_parts();shape=next(s for n,s in parts.items() if n.startswith('J1'))
 shells=[];ex=TopExp_Explorer(shape,TopAbs_SHELL)
 while ex.More():shells.append(TopoDS.Shell_s(ex.Current()));ex.Next()
 incremental=[];individual=[]
 for stop in range(1,len(shells)+1):
  builder=BRepBuilderAPI_MakeSolid(shells[0])
  for s in shells[1:stop]:builder.Add(s)
  incremental.append(dict(added_shell=stop-1,solid_valid=bool(BRepCheck_Analyzer(builder.Solid()).IsValid())))
 for i,shell in enumerate(shells[1:],1):
  builder=BRepBuilderAPI_MakeSolid(shells[0]);builder.Add(shell)
  individual.append(dict(inner_shell=i,outer_plus_this_void_valid=bool(BRepCheck_Analyzer(builder.Solid()).IsValid())))
 report=dict(schema='s50_j1_shell_isolation_v1',incremental_additions=incremental,individual_void_checks=individual,
             caveat='Only source topology is rebuilt for diagnosis. No shell is discarded in collision meshes or original data.')
 (OUT/'j1_shell_isolation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()

