import os,subprocess,sys

def test_gc_reentry_and_concurrent_recording(tmp_path):
    code=r'''
import gc,threading,time
from pathlib import Path
from unloading_sim.workbench import timing as t
assert gc.get_threshold()==(700,10,10)
# Deterministically interrupt an aggregate while it owns the lock.
real=t._bin
once=[False]
def nested(v):
 if not once[0]:
  once[0]=True;gc.collect()
 return real(v)
t._bin=nested
t.aggregate('test','first',.001)
def work():
 for i in range(20000):t.aggregate('test',i,.001)
threads=[threading.Thread(target=work) for _ in range(4)]
for x in threads:x.start()
writers=[]
while any(x.is_alive() for x in threads):
 writers.append(t.write_later(Path(__import__('sys').argv[1])/'timing.json','stress'))
 time.sleep(.01)
for x in threads:x.join()
for x in writers:x.join()
assert once[0]
'''
    subprocess.run([sys.executable,'-c',code,str(tmp_path)],env=dict(os.environ,ECO65_TIMING='1'),check=True,timeout=15)
