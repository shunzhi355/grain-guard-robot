#!/usr/bin/env bash
# Single entry point for the existing Lenovo navigation session.
# Default launch delegates to the established operator-safe bench.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
CONFIG_FILE="$PROJECT_DIR/LENOVO/config/local_robot.operator-live.json"
CHECK_ONLY=0
REQUIRE_LOCAL=0
MAPPING_UI=0
PASSTHROUGH=()

usage() {
    cat <<'EOF'
Usage: bash LENOVO/scripts/start_navigation_session.sh [options]
Default: delegate to start_operator_bench.sh in operator-safe mode.
--check: read-only session check; starts no process.
--require-local: with --check, require local mode and live navigation input.
--mapping-ui: launch the Qt mapping UI in the task-owned Noetic container.
--config PATH: use a specific runtime configuration.
EOF
}

while (($#)); do
    case "$1" in
        --check) CHECK_ONLY=1; shift ;;
        --require-local) REQUIRE_LOCAL=1; shift ;;
        --mapping-ui) MAPPING_UI=1; shift ;;
        --config)
            (($# >= 2)) || { echo "--config requires a path" >&2; exit 2; }
            CONFIG_FILE="$2"
            shift 2
            ;;
        --attended|--external-suction-confirmed)
            PASSTHROUGH+=("$1")
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if (( MAPPING_UI )); then
    exec bash "$PROJECT_DIR/LENOVO/scripts/start_mapping_ui.sh"
fi

if (( CHECK_ONLY )); then
    PYTHON="${PYTHON:-$PROJECT_DIR/.venv/bin/python}"
    [[ -x "$PYTHON" ]] || PYTHON="${PYTHON3:-python3}"
    args=("$PROJECT_DIR/LENOVO/scripts/check_navigation_session.py" "--config" "$CONFIG_FILE")
    (( REQUIRE_LOCAL )) && args+=(--require-local)
    exec env PYTHONPATH="$PROJECT_DIR/3588/src:$PROJECT_DIR/3588:$PROJECT_DIR/LENOVO/src${PYTHONPATH:+:$PYTHONPATH}" \
        "$PYTHON" "${args[@]}"
fi

(( REQUIRE_LOCAL )) && {
    echo "--require-local is valid only with --check" >&2
    exit 2
}
exec bash "$PROJECT_DIR/LENOVO/scripts/start_operator_bench.sh" --config "$CONFIG_FILE" "${PASSTHROUGH[@]}"
