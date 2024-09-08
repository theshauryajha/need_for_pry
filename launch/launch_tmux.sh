#!/bin/bash

# Start a new tmux session named 'drone_race' and detach
tmux new-session -s drone_race -d

# Create and configure the first window for roslaunch
tmux rename-window -t drone_race:0 'roslaunch'
tmux send-keys -t drone_race:0 'source ~/hector_quadrotor_ws/devel/setup.bash' C-m
tmux send-keys -t drone_race:0 'roslaunch drone_race drone_race.launch' C-m

# Split the 'roslaunch' window horizontally
tmux split-window -h -t drone_race:0

sleep 2

# Create and configure the second window for keyboard_teleop
tmux new-window -t drone_race:1 -n 'keyboard_teleop'
tmux send-keys -t drone_race:1 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py' C-m

# Select the 'keyboard_teleop' window to make it active
tmux select-window -t drone_race:1

# Attach to the tmux session
tmux attach-session -t drone_race
