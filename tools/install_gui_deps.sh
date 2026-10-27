#!/usr/bin/env bash
# Only one local Python package; no apt Qt6/SciPy/developer-tool dependency chain.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python3 -c 'from PyQt5 import QtCore; import numpy'
mkdir -p .gui_deps
touch .gui_deps/COLCON_IGNORE
source tools/gui_env.sh
if ! python3 -c 'import pyqtgraph' >/dev/null 2>&1; then
  python3 -m pip install --disable-pip-version-check --no-cache-dir --no-deps \
    --target "$PWD/.gui_deps" pyqtgraph==0.13.4
fi
python3 -c 'import pyqtgraph; from PyQt5.QtCore import PYQT_VERSION_STR; print("PyQt5", PYQT_VERSION_STR, "PyQtGraph", pyqtgraph.__version__)'
