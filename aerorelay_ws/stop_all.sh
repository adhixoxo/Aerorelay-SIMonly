#!/usr/bin/env bash
# Kill every process in the AeroRelay drone stack.
set -x
tmux kill-session -t aerorelay 2>/dev/null
pkill -9 -f "gz sim"
pkill -9 -f "gz-sim"
pkill -9 -f "gz-transport"
pkill -9 -f "ruby"
pkill -9 -f "arducopter"
pkill -9 -f "mavproxy"
pkill -9 -f "mavros"
pkill -9 -f "parameter_bridge"
pkill -9 -f "drone_controller_node"
pkill -9 -f "mission_manager_node"
pkill -9 -f "docking_manager_node"
pkill -9 -f "battery_manager_node"
pkill -9 -f "parcel_visualizer_node"
pkill -9 -f "dashboard_node"
pkill -9 -f "customer_portal_node"
pkill -9 -f "precision_landing_node"
sleep 2
set +x
echo "---- remaining matches (should be empty) ----"
ps aux | grep -E "arducopter|gz|mavros|mavproxy|_node" | grep -v grep

# Optionally kill the whole tmux session too:
# tmux kill-session -t aerorelay
