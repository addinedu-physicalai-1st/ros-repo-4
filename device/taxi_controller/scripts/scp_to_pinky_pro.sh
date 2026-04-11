#!/usr/bin/env bash

set -euo pipefail

# BASH_SOURCE[0] is the current script file.
# dirname gets its folder, and cd && pwd converts it to an absolute path.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_SRC_DIR="${PROJECT_ROOT}/src"

# These defaults match the command you said you usually use:
#   ssh pinky@192.168.4.1
REMOTE_USER="${REMOTE_USER:-pinky}"
REMOTE_HOST="${REMOTE_HOST:-192.168.104.54}"
REMOTE_PORT="${REMOTE_PORT:-22}"
REMOTE_SRC_DIR="${REMOTE_SRC_DIR:-/home/${REMOTE_USER}/pinky_pro/src}"

# Arrays in bash can store multiple values safely.
DRY_RUN=0
PACKAGE_PATHS=()

# function_name() { ... } defines a bash function.
# Think of it like a small reusable command block.
usage() {
  cat <<EOF
Usage:
  $(basename "$0")
  $(basename "$0") --package pinky_pro/pinky_bringup
  $(basename "$0") --package araseo/araseo_navigation --package pinky_pro/pinky_description
  $(basename "$0") --dry-run --package araseo/araseo_controller

Description:
  Copy ROS2 packages from device/taxi_controller/src to the pinky_pro robot with scp.

Options:
  --package REL_PATH  Package path relative to taxi_controller/src.
                      Example: araseo/araseo_navigation
  --dry-run           Print commands without executing them
  -h, --help          Show this help

Behavior:
  If --package is omitted, the whole src directory is copied.
  If --package is used, each package keeps its path under src on the remote side.
EOF
}

log() {
  printf '[scp_to_pinky_pro] %s\n' "$*"
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

# Validate a package path and return it unchanged if it exists.
# This script now accepts only relative paths under src, not absolute paths.
resolve_package_path() {
  local relative_path="$1"
  local full_path="${LOCAL_SRC_DIR}/${relative_path}"

  [[ "${relative_path}" != /* ]] || fail "Use a relative path for --package, not an absolute path: ${relative_path}"
  [[ -n "${relative_path}" ]] || fail "--package requires a value"
  [[ -d "${full_path}" ]] || fail "Package path not found: ${relative_path}"

  printf '%s\n' "${relative_path}"
}

# Run the command normally, or just print it when --dry-run is used.
run_cmd() {
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '[dry-run] '
    printf '%q ' "$@"
    printf '\n'
    return 0
  fi

  "$@"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package)
      [[ $# -ge 2 ]] || fail "--package requires a value"
      PACKAGE_PATHS+=("$(resolve_package_path "$2")")
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

[[ -d "${LOCAL_SRC_DIR}" ]] || fail "Local src directory not found: ${LOCAL_SRC_DIR}"

REMOTE="${REMOTE_USER}@${REMOTE_HOST}"
SSH_CMD=(ssh -p "${REMOTE_PORT}" "${REMOTE}")
SCP_CMD=(scp -P "${REMOTE_PORT}" -r)

log "Local source root: ${LOCAL_SRC_DIR}"
log "Remote source root: ${REMOTE}:${REMOTE_SRC_DIR}"

if [[ "${#PACKAGE_PATHS[@]}" -eq 0 ]]; then
  log "Copying whole src directory"
  run_cmd "${SSH_CMD[@]}" "mkdir -p '${REMOTE_SRC_DIR%/*}'"
  run_cmd "${SCP_CMD[@]}" "${LOCAL_SRC_DIR}" "${REMOTE}:${REMOTE_SRC_DIR%/*}/"
  log "Transfer complete."
  exit 0
fi

# Copy each selected package to the matching parent directory on the robot.
# Example:
#   local  src/araseo/araseo_navigation
#   remote /home/pinky/ros2_ws/src/araseo/araseo_navigation
for relative_path in "${PACKAGE_PATHS[@]}"; do
  local_path="${LOCAL_SRC_DIR}/${relative_path}"
  remote_parent="${REMOTE_SRC_DIR}/$(dirname "${relative_path}")"

  log "Preparing remote directory for ${relative_path}"
  run_cmd "${SSH_CMD[@]}" "mkdir -p '${remote_parent}'"

  log "Copying ${relative_path}"
  run_cmd "${SCP_CMD[@]}" "${local_path}" "${REMOTE}:${remote_parent}/"
done

log "Transfer complete."
