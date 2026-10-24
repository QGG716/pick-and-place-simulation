from pathlib import Path
root=Path.cwd();out=root/'artifacts/m710_l00_top_xplus200_20261024/v2'
s=(root/'artifacts/m710_l00_top_20261024/prepare.py').read_text(encoding='utf-8')
s=s.replace('m710_l00_top_20261024','m710_l00_top_xplus200_20261024/v2')
s=s.replace('0398dc4756df9b97ecbc855fb3fe7f89e26051c9','b05652ea3267863acd12a87b50b3f21e8bf20293')
s=s.replace('work-l00-top-','work-l00-top-xplus200-').replace('l00-top-20261024','l00-top-xplus200-v2-20261024')
s=s.replace('l00-top-source.tar.gz','l00-top-xplus200-v2-source.tar.gz').replace('l00-top-formal.sh','l00-top-xplus200-v2-formal.sh').replace('l00-top-analysis.tar.gz','l00-top-xplus200-v2-analysis.tar.gz')
exec(compile(s,'reused-l00-prepare','exec'))
p=out/'formal-v1.sh';s=p.read_text(encoding='utf-8').replace('--mounting-variant x_minus_500_y_minus_100','--mounting-variant x_minus_300_y_minus_100');p.write_text(s,encoding='utf-8',newline='\n')
p=out/'analyze.py';s=p.read_text(encoding='utf-8')
s=s.replace("  for a in result['attempts']:","  q0=np.array(result['initial']['q_rad'])\n  row['root_configuration']=dict(q_rad=q0.tolist(),q2_rad=float(q0[1]),upper_arm=upper_arm(built.robot,q0))\n  for a in result['attempts']:")
p.write_text(s,encoding='utf-8',newline='\n')
