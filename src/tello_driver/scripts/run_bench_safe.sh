#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-/home/elyes/CATKIN_WS}"

if [ ! -d "$WS_DIR" ]; then
  echo "Workspace not found: $WS_DIR"
  echo "Usage: $0 [CATKIN_WS_PATH]"
  exit 1
fi

cd "$WS_DIR"

if [ ! -f "devel/setup.bash" ]; then
  echo "Missing devel/setup.bash in $WS_DIR"
  echo "Build first: catkin_make --pkg tello_driver"
  exit 1
fi

VENV_PY="$WS_DIR/.venv/bin/python"
VENV_ACTIVATE="$WS_DIR/.venv/bin/activate"

if [ ! -x "$VENV_PY" ] || [ ! -f "$VENV_ACTIVATE" ]; then
  echo "Missing workspace virtualenv in $WS_DIR/.venv"
  echo "Create it first, then install deps (including av)."
  echo "Example:"
  echo "  python3 -m venv $WS_DIR/.venv"
  echo "  source $WS_DIR/.venv/bin/activate"
  echo "  pip install av"
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source "$VENV_ACTIVATE"
set -u

echo "Starting safe bench stack (driver + vision + localization + fake pose)..."
echo "Press Ctrl+C to stop all nodes."

if ! "$VENV_PY" -c "import av, yaml, rospkg" >/dev/null 2>&1; then
  echo "Missing one or more Python modules required by bench stack: av, yaml, rospkg"
  echo "Install in workspace venv, then retry:"
  echo "  source $VENV_ACTIVATE"
  echo "  pip install av PyYAML rospkg catkin_pkg empy"
  exit 1
fi

exec roslaunch tello_driver bench_safe_stack.launch