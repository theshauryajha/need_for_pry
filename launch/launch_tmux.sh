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

# Create and configure the second window for joy_to_twist status
tmux new-window -t drone_race:1 -n 'joy_status'
tmux send-keys -t drone_race:1 'rostopic echo /cmd_vel' C-m

# Select the 'joy_status' window to make it active
tmux select-window -t drone_race:1

# Attach to the tmux session
tmux attach-session -t drone_race