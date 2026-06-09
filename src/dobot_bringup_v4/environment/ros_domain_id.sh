# Set ROS_LOCALHOST_ONLY from WORKSPACE_ROOT/station_config.
# If the legacy robot_bringup JSON still includes ros_domain_id, export it too.

_dobot_find_file_upwards() {
  _dobot_search="$1"
  _dobot_filename="$2"
  while [ -n "$_dobot_search" ] && [ "$_dobot_search" != "/" ]; do
    if [ -f "$_dobot_search/$_dobot_filename" ]; then
      printf '%s\n' "$_dobot_search/$_dobot_filename"
      return 0
    fi
    _dobot_search="$(dirname "$_dobot_search")"
  done
  return 1
}

_dobot_station_config=""
_dobot_ros_domain_config=""

if [ -n "$DOBOT_PICKN_PLACE_ROOT" ]; then
  _dobot_station_config="$DOBOT_PICKN_PLACE_ROOT/station_config"
  _dobot_ros_domain_config="$DOBOT_PICKN_PLACE_ROOT/config/robot_bringup/param.json"
elif [ -n "$DOBOT_WORKSPACE_ROOT" ]; then
  _dobot_station_config="$DOBOT_WORKSPACE_ROOT/station_config"
  _dobot_ros_domain_config="$DOBOT_WORKSPACE_ROOT/config/robot_bringup/param.json"
elif [ -n "$AMENT_CURRENT_PREFIX" ]; then
  _dobot_station_config="$(_dobot_find_file_upwards "$AMENT_CURRENT_PREFIX" "station_config")"
  _dobot_ros_domain_config="$(_dobot_find_file_upwards "$AMENT_CURRENT_PREFIX" "config/robot_bringup/param.json")"
elif [ -n "$COLCON_CURRENT_PREFIX" ]; then
  _dobot_station_config="$(_dobot_find_file_upwards "$COLCON_CURRENT_PREFIX" "station_config")"
  _dobot_ros_domain_config="$(_dobot_find_file_upwards "$COLCON_CURRENT_PREFIX" "config/robot_bringup/param.json")"
fi

if [ -z "$_dobot_ros_domain_config" ] && [ -n "$COLCON_CURRENT_PREFIX" ]; then
  _dobot_ros_domain_config="$COLCON_CURRENT_PREFIX/share/cr_robot_ros2/config/param.json"
fi

if command -v python3 >/dev/null 2>&1; then
  _dobot_ros_env="$(
    python3 - "$_dobot_station_config" "$_dobot_ros_domain_config" <<'PY'
import json
import os
import sys

station_path = sys.argv[1]
json_path = sys.argv[2]


def read_station_config(path):
    settings = {}
    if not path or not os.path.isfile(path):
        return settings
    with open(path, 'r', encoding='utf-8') as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('export '):
                line = line[len('export '):].strip()
            if '=' not in line:
                continue
            key, value = line.split('=', 1)
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
                value = value[1:-1]
            settings[key.strip()] = value
    return settings


def bool_to_ros(value):
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, int) and value in (0, 1):
        return str(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return '1'
        if normalized in ('0', 'false', 'no', 'off'):
            return '0'
    raise SystemExit(
        f'ros_localhost_only must be true/false or 1/0, got {value!r}'
    )


settings = read_station_config(station_path)
data = {}
if json_path and os.path.isfile(json_path):
    with open(json_path, 'r', encoding='utf-8') as stream:
        data = json.load(stream)

domain_value = data.get('ros_domain_id')
if domain_value is None or (isinstance(domain_value, str) and not domain_value.strip()):
    domain_id = ''
else:
    domain_id = int(domain_value)
    if domain_id < 0 or domain_id > 232:
        raise SystemExit(f'ros_domain_id must be between 0 and 232, got {domain_id}')

localhost_value = settings.get('ROS_LOCALHOST_ONLY')
if localhost_value in (None, ''):
    localhost_value = data.get('ros_localhost_only', False)

print(domain_id)
print(bool_to_ros(localhost_value))
PY
  )"
  if [ -n "$_dobot_ros_env" ]; then
    _dobot_ros_domain_id="$(printf '%s\n' "$_dobot_ros_env" | sed -n '1p')"
    _dobot_ros_localhost_only="$(printf '%s\n' "$_dobot_ros_env" | sed -n '2p')"
    if [ -n "$_dobot_ros_domain_id" ]; then
      export ROS_DOMAIN_ID="$_dobot_ros_domain_id"
    fi
    if [ -n "$_dobot_ros_localhost_only" ]; then
      export ROS_LOCALHOST_ONLY="$_dobot_ros_localhost_only"
    fi
  fi
  unset _dobot_ros_env
  unset _dobot_ros_domain_id
  unset _dobot_ros_localhost_only
fi

unset _dobot_station_config
unset _dobot_ros_domain_config
unset -f _dobot_find_file_upwards
