#!/bin/bash
echo "🚀 Booting UAV Rescue Simulation (5-Pane Dashboard)..."

tmux new-session -d -s fyp_dashboard

# --- Create the 5-Pane Grid ---
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 1
tmux split-window -v

# Setup the ROS 2 environment in all 5 panes silently
for i in {0..4}; do
    tmux send-keys -t $i "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash" C-m
done

# --- Pane 0 (Top-Left): Gazebo ---
tmux send-keys -t 0 "export QT_QPA_PLATFORM=xcb" C-m
tmux send-keys -t 0 "export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo:\$GZ_SIM_SYSTEM_PLUGIN_PATH" C-m
tmux send-keys -t 0 "export GZ_SIM_RESOURCE_PATH=/workspace/src/sim_assets/models:/workspace/src/sim_assets/worlds:\$GZ_SIM_RESOURCE_PATH" C-m
tmux send-keys -t 0 "gz sim -v4 -r /workspace/src/sim_assets/worlds/nust.sdf" C-m

# --- Pane 1 (Middle-Left): MAVROS ---
tmux send-keys -t 1 "sleep 8" C-m
tmux send-keys -t 1 "ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14550@" C-m

# --- Pane 2 (Bottom-Left): ROS-GZ Bridge ---
tmux send-keys -t 2 "sleep 12" C-m
tmux send-keys -t 2 "ros2 run ros_gz_bridge parameter_bridge '/world/nust/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' '/world/nust/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' '/world/nust/model/iris_with_gimbal/link/lidar_link/sensor/lidar_2d/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' --ros-args -r /world/nust/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw -r /world/nust/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/depth_image:=/camera/depth_raw -r /world/nust/model/iris_with_gimbal/link/lidar_link/sensor/lidar_2d/scan:=/scan" C-m

# --- Pane 3 (Top-Right): ArduPilot SITL ---
tmux send-keys -t 3 "sleep 5" C-m
tmux send-keys -t 3 "cd /ardupilot/ArduCopter && /ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --add-param-file=/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --console -N --custom-location=33.643479,72.992245,553.0,0 -c 'rc 3 1500'" C-m

# --- Pane 4 (Bottom-Right): AI Vision Controller ---
tmux send-keys -t 4 "sleep 18" C-m
tmux send-keys -t 4 "python3 /workspace/src/disaster_response_ai/disaster_response_ai/vision_controller.py" C-m

# Attach to the dashboard
tmux select-pane -t 4
tmux attach-session -t fyp_dashboard

