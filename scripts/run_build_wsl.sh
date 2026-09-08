#!/usr/bin/env bash
set -euo pipefail
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"
export ROVER_BUILD_JOBS="${ROVER_BUILD_JOBS:-4}"
exec "$(dirname "$0")/configure_s1.sh" "$@"
