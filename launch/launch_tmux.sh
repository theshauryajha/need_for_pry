#!/bin/bash

# Default values
USE_JOY=true

PLAYER_NAME=${1:-"Ghost"}
if [ "$2" == "--keyboard" ]; then
    USE_JOY=false
fi

# Start a new tmux session named 'drone_race' and detach
tmux new-session -s drone_race -d || { echo "Failed to start tmux session"; exit 1; }

# Create and configure the first window for roslaunch
tmux rename-window -t drone_race:0 'roslaunch' || { echo "Failed to rename tmux window"; exit 1; }
tmux send-keys -t drone_race:0 'source ~/hector_quadrotor_ws/devel/setup.bash' C-m 
tmux send-keys -t drone_race:0 "roslaunch drone_race drone_race.launch use_joy:=${USE_JOY} player:=${PLAYER_NAME}" C-m || { echo "Failed to send command to tmux window"; exit 1; }

# Create and configure the second window with teleop_twist_keyboard if required
if [ "$USE_JOY" = false ]; then
    # Wait for roslaunch to complete
    sleep 2  

    # Split the 'roslaunch' window horizontally
    tmux split-window -h -t drone_race:0 || { echo "Failed to split tmux window"; exit 1; }
    tmux send-keys -t drone_race:0 'clear' C-m  # Clear the split window

    # Create a new window for teleop_twist_keyboard
    tmux new-window -t drone_race:1 -n 'teleop_twist_keyboard' || { echo "Failed to create new tmux window"; exit 1; }
    tmux send-keys -t drone_race:1 'source ~/hector_quadrotor_ws/devel/setup.bash' C-m || { echo "Failed to send command to tmux window"; exit 1; }
    tmux send-keys -t drone_race:1 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py' C-m || { echo "Failed to send command to tmux window"; exit 1; }
    
    # Select the appropriate window to make it active
    tmux select-window -t drone_race:1
fi

# Attach to the tmux session
tmux attach-session -t drone_race