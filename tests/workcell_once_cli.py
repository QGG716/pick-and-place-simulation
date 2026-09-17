"""Test-only launcher: substitute expensive boundaries, execute real __main__."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'packages/unloading_contracts/src')]

from pytest import MonkeyPatch
from workcell_once_fakes import SCRIPT, install

if __name__ == '__main__':
    capture = Path(sys.argv[sys.argv.index('--capture') + 1])
    with MonkeyPatch.context() as patch:
        install(patch, capture)
        runpy.run_path(str(SCRIPT), run_name='__main__')
