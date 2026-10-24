from pathlib import Path
root=Path.cwd();out=root/'artifacts/m710_l00_top_20261024'
source='0398dc4756df9b97ecbc855fb3fe7f89e26051c9'
s=(root/'artifacts/m710_c04_side_20261024/prepare.py').read_text(encoding='utf-8')
s=s.replace('m710_c04_side_20261024','m710_l00_top_20261024').replace('94ff46486f4c2a929a9c46323699198a14d18af4',source)
s=s.replace("work='work-c04-side-'","work='work-l00-top-'").replace("round_name='c04-side-20261024'","round_name='l00-top-20261024'")
exec(compile(s,'reused-prepare','exec'))
for name in ('formal-v1.sh','deploy-and-run.sh','analyze.py','analyze.sh'):
    p=out/name;s=p.read_text(encoding='utf-8')
    s=s.replace('--continue-l01-c04-side-from','--continue-l00-top-from').replace('--tool-connector-extension-m 0.1','--tool-connector-extension-m 0.2')
    s=s.replace('c04-side-source.tar.gz','l00-top-source.tar.gz').replace('c04-side-formal.sh','l00-top-formal.sh').replace('c04-side-analysis.tar.gz','l00-top-analysis.tar.gz')
    s=s.replace('pytest -q tests/test_c04_side_hmax.py','pytest -q tests/test_l00_top_continuation.py tests/test_c04_side_hmax.py')
    p.write_text(s,encoding='utf-8',newline='\n')
