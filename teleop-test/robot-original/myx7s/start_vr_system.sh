#!/bin/bash

# VR System Startup Script
# Simple wrapper to start the VR fan-out system

echo "=== VR Robot Control System ==="
echo "Starting fan-out service and controllers..."
echo

# Check if Python files exist
if [ ! -f "vr_fanout.py" ]; then
    echo "Error: vr_fanout.py not found!"
    exit 1
fi

if [ ! -f "vr_base.py" ]; then
    echo "Warning: vr_base.py not found - base controller will not start"
fi

if [ ! -f "vr_dual_arm.py" ]; then
    echo "Warning: vr_dual_arm.py not found - dual arm controller will not start"
fi

# Run the system launcher
python3 run_vr_system.py "$@"