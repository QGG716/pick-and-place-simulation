"""Open an exported, self-contained diagnostic page without an HTTP server."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('html', nargs='?', type=Path,
                        help='Exported HTML; defaults to the newest local diagnostic viewer.')
    args = parser.parse_args()
    if args.html is None:
        candidates = list((ROOT / 'outputs/single_carton_development').glob('viewer*/index.html'))
        if not candidates:
            parser.error('没有已导出的查看器，请先运行 export_eco65_physical_diagnostic_viewer.py')
        path = max(candidates, key=lambda p: p.stat().st_mtime)
    else:
        path = args.html.expanduser().resolve()
    if not path.is_file() or path.suffix.lower() != '.html':
        parser.error(f'查看器文件不存在：{path}')

    # The desktop URL dispatch may accept a request without starting a browser.
    # Invoke Firefox directly when available, without changing default settings.
    firefox = Path('/snap/bin/firefox')
    command = str(firefox) if firefox.exists() else shutil.which('firefox')
    argv = [command, '--new-window', path.as_uri()] if command else None
    if argv is None:
        opener = shutil.which('xdg-open')
        if not opener:
            parser.error(f'未找到浏览器，请手动打开：{path}')
        argv = [opener, path.as_uri()]
    log_path = path.parent / 'browser-launch.log'
    with log_path.open('a') as log:
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=log, start_new_session=True)
    print(f'已请求浏览器打开：{path}\n无需本地服务。启动日志：{log_path}')
    # An existing browser may acknowledge the handoff and exit successfully.
    try:
        code = process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        return
    if code:
        parser.error(f'浏览器启动失败（{code}），请查看：{log_path}')


if __name__ == '__main__':
    main()
