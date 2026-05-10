# Set ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY from
# WORKSPACE_ROOT/config/dobot_bringup_v4/param.json.

_dobot_ros_domain_find_config() {
  _dobot_search="$1"
  while [ -n "$_dobot_search" ] && [ "$_dobot_search" != "/" ]; do
    if [ -f "$_dobot_search/config/dobot_bringup_v4/param.json" ]; then
      printf '%s\n' "$_dobot_search/config/dobot_bringup_v4/param.json"
      return 0
    fi
    _dobot_search="$(dirname "$_dobot_search")"
  done
  return 1
}

_dobot_ros_domain_config=""
if [ -n "$DOBOT_PICKN_PLACE_ROOT" ]; then
  _dobot_ros_domain_config="$DOBOT_PICKN_PLACE_ROOT/config/dobot_bringup_v4/param.json"
elif [ -n "$DOBOT_WORKSPACE_ROOT" ]; then
  _dobot_ros_domain_config="$DOBOT_WORKSPACE_ROOT/config/dobot_bringup_v4/param.json"
elif [ -n "$AMENT_CURRENT_PREFIX" ]; then
  _dobot_ros_domain_config="$(_dobot_ros_domain_find_config "$AMENT_CURRENT_PREFIX")"
elif [ -n "$COLCON_CURRENT_PREFIX" ]; then
  _dobot_ros_domain_config="$(_dobot_ros_domain_find_config "$COLCON_CURRENT_PREFIX")"
fi

if [ -z "$_dobot_ros_domain_config" ] && [ -n "$COLCON_CURRENT_PREFIX" ]; then
  _dobot_ros_domain_config="$COLCON_CURRENT_PREFIX/share/cr_robot_ros2/config/param.json"
fi

if [ -f "$_dobot_ros_domain_config" ] && command -v python3 >/dev/null 2>&1; then
  _dobot_ros_env="$(
    python3 - "$_dobot_ros_domain_config" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, 'r', encoding='utf-8') as stream:
    data = json.load(stream)

domain_id = int(data.get('ros_domain_id', 0))
if domain_id < 0 or domain_id > 232:
    raise SystemExit(f'ros_domain_id must be between 0 and 232, got {domain_id}')

localhost_value = data.get('ros_localhost_only', False)
if isinstance(localhost_value, bool):
    localhost_only = '1' if localhost_value else '0'
elif isinstance(localhost_value, int) and localhost_value in (0, 1):
    localhost_only = str(localhost_value)
elif isinstance(localhost_value, str):
    normalized = localhost_value.strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        localhost_only = '1'
    elif normalized in ('0', 'false', 'no', 'off'):
        localhost_only = '0'
    else:
        raise SystemExit(
            f'ros_localhost_only must be true/false or 1/0, got {localhost_value!r}'
        )
else:
    raise SystemExit(
        f'ros_localhost_only must be true/false or 1/0, got {localhost_value!r}'
    )

print(domain_id)
print(localhost_only)
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

unset _dobot_ros_domain_config
unset -f _dobot_ros_domain_find_config
