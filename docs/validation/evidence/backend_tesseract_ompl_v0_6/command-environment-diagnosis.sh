set -u
cd /root/autodl-tmp/tesseract-ompl-20260922/repo
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
printf 'EXISTING CPU ENVIRONMENT\n'
"$PY" -c 'import pinocchio, coal; print(pinocchio.__version__)'
printf 'WITH NATIVE LIBRARY OVERRIDE\n'
LD_LIBRARY_PATH=/root/autodl-tmp/tesseract-ompl-20260922/native/lib:/root/autodl-tmp/tesseract-ompl-20260922/native/lib64 "$PY" -c 'import pinocchio, coal; print(pinocchio.__version__)'
