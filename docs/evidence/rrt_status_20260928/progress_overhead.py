from pathlib import Path
import json,statistics,time,io
from test_stage_motion_policy import connector, START, GOAL
root=Path.cwd(); evidence=root.parent/'evidence'; c=connector(); obstacles=[]
def run():
 return c._transit(START,GOAL,obstacles,purpose='FREE_APPROACH',stage='pregrasp',seed=44,iteration_budget=200)
assert run()[1] is None
records=[0]; samples={False:[],True:[]}
with (evidence/'progress_overhead_raw.jsonl').open('w') as f:
 def callback(item):
  records[0]+=1; f.write(json.dumps(item)+'\n'); f.flush()
 for repeat in range(5):
  for enabled in ([False,True] if repeat%2==0 else [True,False]):
   c.progress_callback=callback if enabled else None
   start=time.perf_counter()
   for _ in range(20): assert run()[1] is None
   samples[enabled].append((time.perf_counter()-start)/20)
r=dict(scope='ANALYTIC_SAME_BINDING_HOT_DIRECT_WITH_REAL_JSONL_FLUSH',repeats=5,calls_per_repeat=20,
 samples_seconds=samples,median_seconds={str(k):statistics.median(v) for k,v in samples.items()},
 progress_records=records[0],serialized_bytes=(evidence/'progress_overhead_raw.jsonl').stat().st_size,
 rrt_iterations=c._statistics['rrt_iterations_consumed'],context=c.context_statistics)
(evidence/'progress_overhead.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
