import os,sys,time,json
from pathlib import Path
outer=Path('/root/autodl-tmp/m710-static-prior-20261003')
root_pid=int((outer/'formal.pid').read_text())
out=outer/'formal-resources.jsonl'
while not (outer/'formal.exit').exists():
 records={}
 for d in Path('/proc').iterdir():
  if not d.name.isdigit(): continue
  try:
   s=(d/'stat').read_text(); fields=s[s.rfind(')')+2:].split();records[int(d.name)]=(int(fields[1]),fields,d)
  except (OSError,ValueError): pass
 selected={root_pid}; changed=True
 while changed:
  new={pid for pid,(ppid,_,_) in records.items() if ppid in selected}-selected
  changed=bool(new);selected.update(new)
 processes=[]
 for pid in sorted(selected):
  if pid not in records: continue
  ppid,f,d=records[pid]
  try:
   status=(d/'status').read_text().splitlines(); values={x.split(':',1)[0]:x.split(':',1)[1].strip() for x in status}
   processes.append(dict(pid=pid,ppid=ppid,name=values.get('Name'),rss_kib=values.get('VmRSS'),threads=values.get('Threads'),cpu_allowed=values.get('Cpus_allowed_list'),cpu_ticks=int(f[11])+int(f[12])))
  except OSError: pass
 row=dict(time_unix=time.time(),loadavg=Path('/proc/loadavg').read_text().strip(),processes=processes)
 with out.open('a') as stream: stream.write(json.dumps(row)+chr(10))
 time.sleep(5)
