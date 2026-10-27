#!/usr/bin/env bash
# Source after install/setup.bash when PyQtGraph is installed locally.
vehicle_gui_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$vehicle_gui_repo/.gui_deps${PYTHONPATH:+:$PYTHONPATH}"
export PYQTGRAPH_QT_LIB=PyQt5
if [[ -f "$vehicle_gui_repo/.gui_deps/NotoSansCJK-Regular.ttc" ]]; then
  export VEHICLE_GUI_FONT="$vehicle_gui_repo/.gui_deps/NotoSansCJK-Regular.ttc"
fi
unset vehicle_gui_repo
