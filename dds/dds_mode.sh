# DDS mode switcher for local vs multi-PC (Tailscale + Fast DDS Discovery Server).
#
#   source <repo>/dds/dds_mode.sh
#   dds_local    # single-PC dev / simulation (SIMPLE discovery)  -- default in a fresh shell
#   dds_server   # the machine that runs bringup (PC1)
#   dds_client   # action_client.py / ros2 CLI / rqt / rviz (PC2, or PC1 introspection)
#   dds_status   # print the current mode
#
# RULE: terminals that must talk to each other have to share a mode family.
#   LOCAL <-> LOCAL   : OK
#   SERVER <-> CLIENT : OK
#   LOCAL <-> SERVER/CLIENT : they will NOT see each other.
#
# The discovery server itself is started separately on PC1, e.g.:
#   fastdds discovery --server-id 0 -l 0.0.0.0 -p 11811

# Resolve this script's directory so the XML path is absolute regardless of CWD.
if [ -n "${BASH_SOURCE[0]}" ]; then
  _DDS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  _DDS_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"   # zsh fallback
fi

_DDS_SERVER_IP="100.88.157.53"          # PC1 Tailscale IP (discovery server host)
_DDS_SERVER="${_DDS_SERVER_IP}:11811"
_DDS_PROFILE="${_DDS_DIR}/super_client.xml"
_DDS_DOMAIN=25

dds_local() {
  unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_DOMAIN_ID=${_DDS_DOMAIN}
  export ROS_LOCALHOST_ONLY=0
  ros2 daemon stop >/dev/null 2>&1
  echo "[dds] LOCAL  (simple discovery, domain ${_DDS_DOMAIN})"
}

dds_server() {
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_DOMAIN_ID=${_DDS_DOMAIN}
  export ROS_LOCALHOST_ONLY=0
  export ROS_DISCOVERY_SERVER="${_DDS_SERVER}"
  ros2 daemon stop >/dev/null 2>&1
  echo "[dds] SERVER (discovery-server client -> ${_DDS_SERVER}, domain ${_DDS_DOMAIN})"
}

dds_client() {
  unset ROS_DISCOVERY_SERVER
  export ROS_DOMAIN_ID=${_DDS_DOMAIN}
  export ROS_LOCALHOST_ONLY=0
  export FASTRTPS_DEFAULT_PROFILES_FILE="${_DDS_PROFILE}"
  ros2 daemon stop >/dev/null 2>&1
  echo "[dds] CLIENT (super_client -> ${_DDS_SERVER}, domain ${_DDS_DOMAIN})"
  [ -f "${_DDS_PROFILE}" ] || echo "[dds] WARNING: profile not found: ${_DDS_PROFILE}"
}

dds_status() {
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-<unset>}"
  echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
  if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then echo "  -> mode: CLIENT (super_client)"
  elif [ -n "${ROS_DISCOVERY_SERVER}" ];        then echo "  -> mode: SERVER"
  else                                                echo "  -> mode: LOCAL"
  fi
}

# Convenience: `source dds_mode.sh <mode>` applies that mode immediately, so you
# don't have to source and then call the function on a second line. Both work:
#   source dds/dds_mode.sh server
#   source dds/dds_mode.sh          # then: dds_server
if [ -n "$1" ]; then
  case "$1" in
    local|dds_local)   dds_local ;;
    server|dds_server) dds_server ;;
    client|dds_client) dds_client ;;
    status|dds_status) dds_status ;;
    *) echo "[dds] unknown mode: '$1' (use: local | server | client | status)" ;;
  esac
fi
