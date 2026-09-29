#!/usr/bin/env bash

MAPPING_DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_PROJECT_ROOT="$(cd "${MAPPING_DEPLOY_DIR}/../.." && pwd)"

mapping_source_setup() {
    local setup="$1"
    local nounset_was_enabled=0
    local source_status

    case "$-" in
        *u*) nounset_was_enabled=1; set +u ;;
    esac
    # Catkin-generated setup files may read variables such as ROS_DISTRO before
    # defining them, so source them without inheriting a caller's nounset mode.
    # shellcheck disable=SC1090
    source "$setup"
    source_status=$?
    [ "$nounset_was_enabled" -eq 0 ] || set -u
    return "$source_status"
}

mapping_load_config() {
    local env_file="${MAPPING_ENV_FILE:-${MAPPING_DEPLOY_DIR}/mapping.env}"
    local app_config="${MAPPING_APP_CONFIG:-${MAPPING_PROJECT_ROOT}/config/livox_config.json}"

    [ -f "$env_file" ] || { echo "[FAIL] mapping env missing: $env_file" >&2; return 1; }
    # shellcheck disable=SC1090
    source "$env_file"
    [ -f "$app_config" ] || { echo "[FAIL] application mapping config missing: $app_config" >&2; return 1; }
    command -v python3 >/dev/null 2>&1 || { echo "[FAIL] python3 is required to read $app_config" >&2; return 1; }

    if [ -z "${MID360_IP:-}" ]; then
        MID360_IP="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["MID360_IP"])' "$app_config")" || return 1
    fi
    if [ -z "${RADAR_HOST_IP:-}" ]; then
        RADAR_HOST_IP="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["HOST_IP"])' "$app_config")" || return 1
    fi

    export MAPPING_ENV_FILE="$env_file" MAPPING_APP_CONFIG="$app_config"
    export MID360_IP RADAR_HOST_IP RADAR_NETDEV LIVOX_WS SFAST_WS
    export LIVOX_PACKAGE LIVOX_LAUNCH LIVOX_LIDAR_TOPIC LIVOX_IMU_TOPIC
    export SFAST_PACKAGE SFAST_MAPPING_EXECUTABLE SFAST_CONFIG SFAST_RELOCALIZATION_LAUNCH
    export MAPPING_TOPIC_WAIT_SECONDS
}

mapping_source_ros1() {
    if [ "${ROS_VERSION:-}" = "2" ]; then
        echo "[FAIL] ROS2 environment is active; this project requires ROS1" >&2
        return 1
    fi
    if command -v roscore >/dev/null 2>&1 && command -v roslaunch >/dev/null 2>&1 && command -v rostopic >/dev/null 2>&1; then
        return 0
    fi

    local setup
    if [ -n "${ROS_SETUP_BASH:-}" ]; then
        [ -f "$ROS_SETUP_BASH" ] || { echo "[FAIL] ROS_SETUP_BASH not found: $ROS_SETUP_BASH" >&2; return 1; }
        mapping_source_setup "$ROS_SETUP_BASH"
    else
        for setup in /opt/ros/*/setup.bash; do
            [ -f "$setup" ] || continue
            mapping_source_setup "$setup"
            if [ "${ROS_VERSION:-1}" != "2" ] && command -v roscore >/dev/null 2>&1; then
                ROS_SETUP_BASH="$setup"
                export ROS_SETUP_BASH
                break
            fi
        done
    fi

    command -v roscore >/dev/null 2>&1 &&
        command -v roslaunch >/dev/null 2>&1 &&
        command -v rostopic >/dev/null 2>&1 || {
            echo "[FAIL] ROS1 commands unavailable (roscore, roslaunch, rostopic)" >&2
            return 1
        }
}

mapping_source_workspace() {
    local workspace="$1"
    local setup
    for setup in "$workspace/devel/setup.bash" "$workspace/install/setup.bash"; do
        if [ -f "$setup" ]; then
            mapping_source_setup "$setup"
            return 0
        fi
    done
    echo "[FAIL] workspace setup not found under $workspace" >&2
    return 1
}

mapping_route_is_exact() {
    local route
    route="$(ip route get "$MID360_IP" 2>/dev/null | head -n 1)" || return 1
    printf '%s\n' "$route" | grep -Eq "(^| )${MID360_IP//./\\.}( |$)" &&
        printf '%s\n' "$route" | grep -Eq "(^| )dev ${RADAR_NETDEV}( |$)" &&
        printf '%s\n' "$route" | grep -Eq "(^| )src ${RADAR_HOST_IP//./\\.}( |$)"
}

mapping_wait_for_topic() {
    local topic="$1"
    local max_wait="${2:-$MAPPING_TOPIC_WAIT_SECONDS}"
    local start
    start="$(date +%s)"
    until timeout 3 rostopic hz "$topic" 2>/dev/null | grep -q "average rate"; do
        if [ "$(( $(date +%s) - start ))" -ge "$max_wait" ]; then
            echo "[FAIL] $topic has no data after ${max_wait}s" >&2
            return 1
        fi
        sleep 1
    done
    echo "[PASS] $topic is publishing"
}
