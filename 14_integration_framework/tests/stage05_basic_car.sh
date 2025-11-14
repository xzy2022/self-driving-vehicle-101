#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export SELF_DRIVING_REPO_ROOT="${REPO_ROOT}"

roslaunch simulation_bringup stage05_basic_car.launch session_mode:=test "$@"
