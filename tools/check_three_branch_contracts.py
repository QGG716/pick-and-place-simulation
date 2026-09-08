"""Check fixed consumer SHAs in detached worktrees without changing branches."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def run(command, *, cwd):
    return subprocess.run(command, cwd=cwd, check=True, text=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, default=ROOT)
    parser.add_argument("--check-only", action="store_true", help="only verify that patches apply to the fixed SHAs")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "integration" / "manifest.json").read_text(encoding="utf-8"))
    for name, consumer in manifest["consumers"].items():
        commit = consumer["commit"]
        patch = ROOT / consumer["patch"]
        run(["git", "cat-file", "-e", commit + "^{commit}"], cwd=args.repository)
        with tempfile.TemporaryDirectory(prefix="unloading-contract-") as temporary:
            worktree = Path(temporary) / name
            run(["git", "worktree", "add", "--detach", str(worktree), commit], cwd=args.repository)
            try:
                run(["git", "apply", "--check", str(patch)], cwd=worktree)
                if not args.check_only:
                    run(["git", "apply", str(patch)], cwd=worktree)
                    run(["python3", "-m", "pip", "install", str(ROOT / "packages" / "unloading_contracts")], cwd=worktree)
                    if name == "online_continuous":
                        run(["python3", "-m", "pytest", "-q", "tests/test_online_backend_contracts.py", "tests/test_online_planning.py"], cwd=worktree)
                    else:
                        run(["python3", "-m", "compileall", "-q", "src/unloading_sim/feasibility_contract_adapter.py"], cwd=worktree)
            finally:
                run(["git", "worktree", "remove", "--force", str(worktree)], cwd=args.repository)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
