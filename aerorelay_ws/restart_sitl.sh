#!/usr/bin/env bash
set -e

SESSION="aerorelay"
ARDUPILOT_DIR="$HOME/AeroRelay/ardupilot"

echo "Restarting SITL windows..."

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Error: tmux session '$SESSION' not found."
    exit 1
fi

if [[ ! -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]]; then
    echo "Error: ArduCopter SITL binary not found."
    echo "Expected: $ARDUPILOT_DIR/build/sitl/bin/arducopter"
    exit 1
fi

tmux respawn-window -k -t "$SESSION:sitl1" \
"cd '$ARDUPILOT_DIR' && ./build/sitl/bin/arducopter --model JSON --sim-address=127.0.0.1:9002 -I0 --sysid 1"

tmux respawn-window -k -t "$SESSION:sitl2" \
"cd '$ARDUPILOT_DIR' && ./build/sitl/bin/arducopter --model JSON --sim-address=127.0.0.1:9012 -I1 --sysid 2"

echo "Waiting for Drone 1..."
until ros2 topic echo /drone1/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done

echo "Drone 1 connected"

echo "Waiting for Drone 2..."
until ros2 topic echo /drone2/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done

echo "Drone 2 connected"

echo "Waiting for ArduPilot plugin to stabilize..."
sleep 5

echo "===================================="
echo "READY FOR NEXT MISSION"
echo "===================================="
