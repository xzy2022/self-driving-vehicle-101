#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "SCRIPT_DIR: ${SCRIPT_DIR}"

REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
echo "REPO_ROOT: ${REPO_ROOT}"

export SELF_DRIVING_REPO_ROOT="${REPO_ROOT}"
echo "SELF_DRIVING_REPO_ROOT: ${SELF_DRIVING_REPO_ROOT}"

roslaunch simulation_bringup stage04_box_demo.launch session_mode:=test "$@"