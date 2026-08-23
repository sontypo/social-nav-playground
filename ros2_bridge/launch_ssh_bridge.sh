#!/usr/bin/env bash
# ==============================================================================
# SocialNav Studio - Remote SSH Bridge Gateway Launcher
# Launches the zero-dependency Python WebSocket SSH server on ws://localhost:9092
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

echo -e "\033[1;36m=======================================================================\033[0m"
echo -e "\033[1;32m  SocialNav Studio - Remote SSH Bridge & Tunnel Gateway\033[0m"
echo -e "  Port: \033[1;33m9092\033[0m (WebSocket IPC) <==> Remote Robot SSH (Port 22)"
echo -e "\033[1;36m=======================================================================\033[0m"

python3 "${SCRIPT_DIR}/remote_ssh_manager.py"
