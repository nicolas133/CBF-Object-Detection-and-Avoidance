#!/bin/bash

# Start Lidar and other important car drivers using the included launch file
gnome-terminal -- bash -c "roslaunch yahboomcar_nav sick.launch; exec bash" &

# Start PID controller node
gnome-terminal -- bash -c " sleep 5; rosrun toTarget PID_communicator1.py; exec bash" &

# Start Clustering and Wrapping algorithm node
gnome-terminal -- bash -c " sleep 5; rosrun sensor_stick test.py; exec bash" &

# Start CBF filter node with delay
gnome-terminal -- bash -c "sleep 7; rosrun toTarget cbf_filter1.py; exec bash" &

# Wait for all background processes to complete
wait

