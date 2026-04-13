#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${SCRIPT_DIR}/config.yaml"

DRY_RUN=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0")
  $(basename "$0") --dry-run

Description:
  Read ${CONFIG_PATH} and copy the configured files/directories to the pinky_pro robot with scp.
  The remote password is also read from config.yaml and passed through sshpass.

Options:
  --dry-run   Print commands without executing them
  -h, --help  Show this help
EOF
}

log() {
  printf '[scp_to_pinky_pro] %s\n' "$*"
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

run_cmd() {
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '[dry-run] '
    printf '%q ' "$@"
    printf '\n'
    return 0
  fi

  "$@"
}

run_remote_cmd() {
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '[dry-run] '
    printf '%q ' "${SSH_CMD[@]}"
    printf '%q' "$1"
    printf '\n'
    return 0
  fi

  SSHPASS="${REMOTE_PASSWORD}" sshpass -e "${SSH_CMD[@]}" "$1"
}

run_copy_cmd() {
  local local_path="$1"
  local remote_dest="$2"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '[dry-run] '
    printf '%q ' "${RSYNC_CMD[@]}" "${EXCLUDES[@]}" "${local_path}" "${remote_dest}"
    printf '\n'
    return 0
  fi

  SSHPASS="${REMOTE_PASSWORD}" sshpass -e "${RSYNC_CMD[@]}" "${EXCLUDES[@]}" "${local_path}" "${remote_dest}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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

[[ -f "${CONFIG_PATH}" ]] || fail "Config file not found: ${CONFIG_PATH}"

CONFIG_LINES="$(python3 - "${CONFIG_PATH}" "${PROJECT_ROOT}" <<'PY'
import sys
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError as exc:
    raise SystemExit(f"PyYAML is required to read config.yaml: {exc}")

config_path = Path(sys.argv[1])
project_root = Path(sys.argv[2]).resolve()

data = yaml.safe_load(config_path.read_text()) or {}
remote = data.get("remote") or {}
packages = data.get("packages") or []

user = remote.get("user")
host = remote.get("host")
port = remote.get("port")
password = remote.get("password")

if not user:
    raise SystemExit("config.yaml: remote.user is required")
if not host:
    raise SystemExit("config.yaml: remote.host is required")
if port is None:
    raise SystemExit("config.yaml: remote.port is required")
if password is None:
    raise SystemExit("config.yaml: remote.password is required")
if not isinstance(packages, list) or not packages:
    raise SystemExit("config.yaml: packages must contain at least one entry")

print(f"REMOTE_USER={user}")
print(f"REMOTE_HOST={host}")
print(f"REMOTE_PORT={port}")
print(f"REMOTE_PASSWORD={password!r}")

for index, pkg in enumerate(packages, start=1):
    if not isinstance(pkg, dict):
        raise SystemExit(f"config.yaml: packages[{index}] must be a mapping")

    name = pkg.get("name") or f"package_{index}"
    input_path_raw = pkg.get("input_path")
    output_path = pkg.get("output_path")

    if not input_path_raw:
      raise SystemExit(f"config.yaml: packages[{index}].input_path is required")
    if not output_path:
      raise SystemExit(f"config.yaml: packages[{index}].output_path is required")
    if not str(output_path).startswith("/"):
      raise SystemExit(f"config.yaml: packages[{index}].output_path must be an absolute path")

    input_path = (project_root / str(input_path_raw)).resolve()
    try:
        input_path.relative_to(project_root)
    except ValueError:
        raise SystemExit(
            f"config.yaml: packages[{index}].input_path must stay inside the project: {input_path_raw}"
        )

    if not input_path.exists():
        raise SystemExit(f"config.yaml: input_path not found: {input_path_raw}")

    print(f"PACKAGE_NAME[{index}]={name}")
    print(f"PACKAGE_INPUT[{index}]={input_path}")
    print(f"PACKAGE_OUTPUT[{index}]={output_path}")
PY
)" || fail "Failed to parse ${CONFIG_PATH}"

eval "${CONFIG_LINES}"

REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

if [[ "${DRY_RUN}" -ne 1 ]]; then
  command -v sshpass >/dev/null 2>&1 || fail "sshpass is required for password-based transfer"
  command -v rsync >/dev/null 2>&1 || fail "rsync is required for optimized transfer"
fi

SSH_CMD=(ssh -o StrictHostKeyChecking=no -p "${REMOTE_PORT}" "${REMOTE}")
RSYNC_CMD=(rsync -avz --delete -e "ssh -o StrictHostKeyChecking=no -p ${REMOTE_PORT}")

EXCLUDES=(
  --exclude='__pycache__'
  --exclude='*.pyc'
  --exclude='*.pyo'
  --exclude='.git'
  --exclude='.vscode'
  --exclude='.idea'
  --exclude='.DS_Store'
  --exclude='build'
  --exclude='install'
  --exclude='log'
)

log "Using config: ${CONFIG_PATH}"
log "Remote target: ${REMOTE}"

package_count=0
for var_name in "${!PACKAGE_INPUT[@]}"; do
  ((package_count += 1))
done

(( package_count > 0 )) || fail "No packages found in ${CONFIG_PATH}"

for index in $(printf '%s\n' "${!PACKAGE_INPUT[@]}" | sort -n); do
  package_name="${PACKAGE_NAME[$index]}"
  local_path="${PACKAGE_INPUT[$index]}"
  remote_path="${PACKAGE_OUTPUT[$index]}"
  remote_parent="$(dirname "${remote_path}")"

  log "Transferring ${package_name}: ${local_path} -> ${REMOTE}:${remote_path}"
  run_remote_cmd "mkdir -p '${remote_parent}'"
  run_copy_cmd "${local_path}" "${REMOTE}:${remote_parent}/"
done

log "Transfer complete."
