import subprocess,time,resource,json,sys,datetime
from pathlib import Path
O=Path('/root/autodl-tmp/m710-static-prior-20261003')
A=Path('/root/autodl-tmp/m710-moveit2-20260922/rootfs/tmp/static-prior-20261003')
assert not (A/'formal-once').exists()
start=time.perf_counter();stamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
status=subprocess.call(['taskset','-c','0-21','chroot','/root/autodl-tmp/m710-moveit2-20260922/rootfs','/bin/bash','/tmp/static-prior-20261003/run-formal.sh'])
usage=resource.getrusage(resource.RUSAGE_CHILDREN)
(O/'formal-process-usage.json').write_text(json.dumps(dict(started_utc=stamp,exit_code=status,process_wall_s=time.perf_counter()-start,cpu_user_s=usage.ru_utime,cpu_system_s=usage.ru_stime,max_child_rss_kib=usage.ru_maxrss,scope='Entire fresh Python invocation and native worker; includes interpreter imports, initialization, planning, scene updates and output delivery.'),indent=2)+chr(10))
(O/'formal.exit').write_text(str(status)+chr(10))
sys.exit(status)
