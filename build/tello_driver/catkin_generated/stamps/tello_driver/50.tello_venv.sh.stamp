# Auto-activate workspace virtual environment when sourcing catkin setup files.
# CATKIN_ENV_HOOK_WORKSPACE is provided by catkin and points to workspace root.

if [ -n "${CATKIN_ENV_HOOK_WORKSPACE:-}" ]; then
  _tello_ws_root="${CATKIN_ENV_HOOK_WORKSPACE:-}"
  _tello_venv_path=""
  if [ -f "$_tello_ws_root/.venv/bin/activate" ]; then
    _tello_venv_path="$_tello_ws_root/.venv"
  elif [ -f "$_tello_ws_root/../.venv/bin/activate" ]; then
    _tello_ws_root="$(cd "$_tello_ws_root/.." 2>/dev/null && pwd)"
    _tello_venv_path="$_tello_ws_root/.venv"
  fi
  if [ -f "$_tello_venv_path/bin/activate" ]; then
    if [ -z "${VIRTUAL_ENV:-}" ] || [ "${VIRTUAL_ENV:-}" != "$_tello_venv_path" ]; then
      if [ -n "${VIRTUAL_ENV:-}" ] && command -v deactivate >/dev/null 2>&1; then
        deactivate >/dev/null 2>&1
      fi
      . "$_tello_venv_path/bin/activate"
    fi
  fi
  unset _tello_ws_root
  unset _tello_venv_path
fi
