#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

SESSION="auv"

# Kill existing session if any
tmux kill-session -t $SESSION 2>/dev/null

# Create new session
tmux new-session -d -s $SESSION -n 'pico1'

# === FIRST RUN ===
echo "First pico flash..."
tmux send-keys -t $SESSION:pico1 "$SCRIPT_DIR/program_pico.sh" C-m

echo "Waiting 10 seconds..."
sleep 10

echo "Launching first UART bridge..."
tmux new-window -t $SESSION -n 'uart1'
tmux send-keys -t $SESSION:uart1 "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 run pico_UART uart_ros_bridge" C-m

echo "Waiting 10 seconds..."
sleep 10

# === SECOND RUN ===
echo "Second pico flash..."
tmux new-window -t $SESSION -n 'pico2'
tmux send-keys -t $SESSION:pico2 "$SCRIPT_DIR/program_pico.sh" C-m

echo "Waiting 10 seconds..."
sleep 10

echo "Launching second UART bridge..."
tmux new-window -t $SESSION -n 'uart2'
tmux send-keys -t $SESSION:uart2 "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 run pico_UART uart_ros_bridge" C-m

echo "Waiting 10 seconds..."
sleep 10

# === MAIN STACK ===
echo "Launching prequal..."
tmux new-window -t $SESSION -n 'prequal'
tmux send-keys -t $SESSION:prequal "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 launch ros_controls prequal_full.launch.py" C-m

# Attach
tmux attach -t $SESSION
