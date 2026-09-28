"""Package immutable raw receipts after the single allowed normal run exits."""
import hashlib,json,pathlib,tarfile
root=pathlib.Path(__file__).resolve().parent
run=json.loads((root/'single_carton_run_192f139.json').read_text())
assert 'exit_code' in run, 'normal attempt is still running'
assert json.loads((root/'single_carton_192f139.json').read_text())['status']!='RUNNING'
exclude={'allocation_tests.txt','borrowed_interrupt_tests.txt','cpu_launcher.log','single_carton_launcher.log','raw_manifest.json'}
files=sorted(p for p in root.rglob('*') if p.is_file() and p.name not in exclude and 'first_feasible' not in p.parts)
server_only=sorted((root/'single_carton_192f139_delivery/first_feasible').glob('*'))
manifest={'server_evidence_directory':str(root),'implementation_commit':run['commit'],'files':{str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in files}}
manifest['server_only_duplicate_baseline']={str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size,'same_as':str(p.parent.parent.relative_to(root)/p.name)} for p in server_only}
assert all(p.read_bytes()==(p.parent.parent/p.name).read_bytes() for p in server_only)
manifest_path=root/'raw_manifest.json'
manifest_path.write_text(json.dumps(manifest,indent=2))
archive=root.parent/'loaded-candidate-evidence.tar.gz'
with tarfile.open(archive,'w:gz') as tar:
 for p in files+[manifest_path]:tar.add(p,arcname=str(p.relative_to(root)))
print(archive,archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest())
