#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

switch_mode=""
if [[ "${1:-}" == --switch ]]; then
    if [[ $# -lt 2 ]]; then
        echo "Usage: $0 [--switch <wifi|ap>]" >&2
        exit 2
    fi
    switch_mode="$2"
    shift 2
    case "$switch_mode" in
        wifi|ap) ;;
        *) echo "Unknown network mode: $switch_mode" >&2; exit 2 ;;
    esac
fi
if [[ $# -ne 0 ]]; then
    echo "Usage: $0 [--switch <wifi|ap>]" >&2
    exit 2
fi
go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_app="$(cd "$go2_script_dir/.." && pwd)"
if [[ -f "$go2_app/config/robot.yaml" ]]; then
    go2_share="$go2_app"
    if [[ -z "${SROBOTIS_ROOT:-}" && -f "$go2_app/../../../build/envsetup.sh" ]]; then
        SROBOTIS_ROOT="$(cd "$go2_app/../../.." && pwd)"
    fi
else
    go2_prefix="$(cd "$go2_script_dir/../.." && pwd)"
    go2_share="$go2_prefix/share/unitree_go2"
fi
local_config="${GO2_LOCAL_CONFIG:-/etc/unitree-go2/robot.yaml}"
if [[ ! -r "$local_config" ]]; then
    echo "Missing board-local configuration: $local_config" >&2
    exit 1
fi

if [[ -n "$switch_mode" ]]; then
    if [[ $EUID -ne 0 ]]; then
        echo "Run this script as root on the Go2 board" >&2
        exit 1
    fi
    service="go2.service"
    systemctl cat "$service" >/dev/null
    dropin="/etc/systemd/system/$service.d"
    install -d -m 755 "$dropin"
    printf '[Service]\nEnvironment=GO2_NETWORK_MODE=%s\n' "$switch_mode" > "$dropin/network.conf"
    chmod 644 "$dropin/network.conf"
    systemctl daemon-reload
    systemctl restart --no-block "$service"
    echo "$service: selected $switch_mode for this run and future boots; restart requested"
    exit 0
fi

network_mode="${GO2_NETWORK_MODE:-wifi}"
case "$network_mode" in
    wifi|ap) ;;
    *) echo "Unknown network mode: $network_mode" >&2; exit 2 ;;
esac

if [[ -n "${SROBOTIS_ROOT:-}" ]]; then
    export SROBOTIS_ROOT
    cd "$SROBOTIS_ROOT"
    set +u
    # shellcheck disable=SC1091
    source build/envsetup.sh
    # shellcheck disable=SC1091
    source output/staging/setup.bash
    set -u
else
    if ! command -v ros2 >/dev/null; then
        echo "ROS 2 is not configured; source its setup.bash or set SROBOTIS_ROOT" >&2
        exit 1
    fi
    if [[ ! -r "$go2_share/local_setup.bash" ]]; then
        echo "Missing Go2 package setup: $go2_share/local_setup.bash" >&2
        exit 1
    fi
    set +u
    # shellcheck disable=SC1091
    source "$go2_share/local_setup.bash"
    set -u
fi

exec ros2 launch unitree_go2 go2.launch.py \
    local_config:="$local_config" \
    backend:=serial \
    sbus:=true \
    web:=true \
    web_port:=8082 \
    network_mode:="$network_mode" \
    motion_log_path:=/dev/null
