#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source env.sh
falcon_log_dir="${FALCON_LOG_DIR:-$FALCON_WORKSPACE/logs/office_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$falcon_log_dir/instrumentation" "$falcon_log_dir/turn_trace"
export FALCON_REPAIR_STAGE=A
export FALCON_TARGETED_DIAG_DIR="$falcon_log_dir/instrumentation"
export FALCON_TURN_TRACE_DIR="$falcon_log_dir/turn_trace"
exec roslaunch falcon_native office.launch depth_mode:=official_compatible "log_dir:=$falcon_log_dir" "$@"
