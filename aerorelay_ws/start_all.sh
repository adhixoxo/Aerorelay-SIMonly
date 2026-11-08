#!/usr/bin/env bash
# One-command launcher for the AeroRelay dual-drone stack.
# Creates a tmux session with one window per process, each window
# waits (via `sleep`) for its dependencies before running its command,
# so you don't have to time the terminals by hand.
#
# Usage:
#   ./start_all.sh          # create + attach
#   tmux attach -t aerorelay # reattach later if you detach with Ctrl-b d
#
# Tune the DELAY_* values below if things start before their deps are ready.

set -e

SESSION="aerorelay"

# ---- tune these (seconds) ----------------------------------------------
DELAY_SITL=20
DELAY_MAVPROXY=35
DELAY_MAVROS=50
DELAY_BRIDGES=60
DELAY_CONTROLLERS=70
DELAY_MANAGERS=80
DELAY_APPS=90        # dashboard/parcel_visualizer/customer_portal
# -------------------------------------------------------------------------

ROS_SETUP="source /opt/ros/jazzy/setup.bash && source ~/AeroRelay/aerorelay_ws/install/setup.bash"
DEACTIVATE="deactivate 2>/dev/null; true"

kill_existing() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists. Attaching to it instead of relaunching."
    echo "Run ./stop_all.sh first if you want a clean restart."
    tmux attach -t "$SESSION"
    exit 0
  fi
}

new_window() {
  local name="$1"; local cmd="$2"
  tmux new-window -t "$SESSION" -n "$name" "bash -c '$cmd; exec bash'"
}

kill_existing

tmux new-session -d -s "$SESSION" -n gazebo \
  "bash -c 'cd ~/AeroRelay/ardupilot_gazebo && taskset -c 0-3 gz sim -v4 -r --headless-rendering worlds/iris_runway_dual.sdf; exec bash'"

new_window "sitl1" \
  "sleep $DELAY_SITL; cd ~/AeroRelay/ardupilot && taskset -c 4 ./build/sitl/bin/arducopter --model JSON --sim-address=127.0.0.1:9002 -I0 --sysid 1; exec bash"

new_window "sitl2" \
"sleep $DELAY_SITL; cd ~/AeroRelay/ardupilot && taskset -c 5 ./build/sitl/bin/arducopter --model JSON --sim-address=127.0.0.1:9012 -I1 --sysid 2; exec bash"

new_window "mavproxy1" \
  "sleep $DELAY_MAVPROXY; taskset -c 6 mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550"

new_window "mavproxy2" \
  "sleep $DELAY_MAVPROXY; taskset -c 7 mavproxy.py --master=tcp:127.0.0.1:5770 --out=udp:127.0.0.1:14560"

new_window "clock_bridge" \
  "sleep $DELAY_MAVROS; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"

new_window "camera1" \
"sleep $DELAY_MAVROS; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge \
/drone1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
/drone1/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"


new_window "camera2" \
"sleep $DELAY_MAVROS; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge \
/drone2/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
/drone2/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"

new_window "mavros1" \
  "sleep $DELAY_MAVROS; $DEACTIVATE; source /opt/ros/jazzy/setup.bash; taskset -c 8 ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 namespace:=drone1 tgt_system:=1"

new_window "mavros2" \
  "sleep $DELAY_MAVROS; $DEACTIVATE; source /opt/ros/jazzy/setup.bash; taskset -c 9 ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14560@14565 namespace:=drone2 tgt_system:=2"

new_window "lidar1" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /drone1/obstacle_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /drone1/obstacle_lidar:=/drone1/obstacle_lidar"

new_window "lidar2" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /drone2/obstacle_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /drone2/obstacle_lidar:=/drone2/obstacle_lidar"

new_window "servo_cmd1" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /drone1/lidar_servo_cmd@std_msgs/msg/Float64]gz.msgs.Double --ros-args -r /drone1/lidar_servo_cmd:=/drone1/lidar_servo_cmd"

new_window "servo_cmd2" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /drone2/lidar_servo_cmd@std_msgs/msg/Float64]gz.msgs.Double --ros-args -r /drone2/lidar_servo_cmd:=/drone2/lidar_servo_cmd"

new_window "servo_pos1" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_drone1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model --ros-args -r /world/iris_runway/model/iris_drone1/joint_state:=/drone1/joint_states"

new_window "servo_pos2" \
"sleep $DELAY_BRIDGES; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_drone2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model --ros-args -r /world/iris_runway/model/iris_drone2/joint_state:=/drone2/joint_states"

new_window "apriltag1" \
"sleep $DELAY_BRIDGES; \
source /opt/ros/jazzy/setup.bash; \
source ~/AeroRelay/aerorelay_ws/install/setup.bash; \
source ~/AeroRelay/venv/bin/activate; \
export PYTHONPATH="$HOME/AeroRelay/venv/lib/python3.12/site-packages:$PYTHONPATH"; \
cd ~/AeroRelay/aerorelay_ws; \
ros2 run aerorelay_apriltag_detector apriltag_detector_node \
--ros-args \
-r __ns:=/drone1 \
-r /camera/image_raw:=/drone1/camera/image_raw \
-r /camera/camera_info:=/drone1/camera/camera_info"

new_window "precision1" \
"sleep $DELAY_CONTROLLERS; \
source /opt/ros/jazzy/setup.bash; \
source ~/AeroRelay/aerorelay_ws/install/setup.bash; \
ros2 launch aerorelay_precision_landing precision_landing.launch.py namespace:=drone1 drone_id:=drone1"

new_window "ctrl1" \
  "sleep $DELAY_CONTROLLERS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_drone_controller drone_controller.launch.py drone_id:=drone1 namespace:=drone1"

new_window "ctrl2" \
  "sleep $DELAY_CONTROLLERS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_drone_controller drone_controller.launch.py drone_id:=drone2 namespace:=drone2"

new_window "mission_mgr" \
  "sleep $DELAY_MANAGERS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_mission_manager mission_manager.launch.py drone1_id:=drone1 drone2_id:=drone2"

new_window "battery_mgr" \
  "sleep $DELAY_MANAGERS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_battery_manager battery_manager.launch.py drone1_id:=drone1 drone2_id:=drone2"

new_window "docking_mgr" \
  "sleep $DELAY_MANAGERS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_docking_manager docking_manager.launch.py drone1_id:=drone1 drone2_id:=drone2 drone1_dock_x:=-0.5 drone1_dock_y:=-0.5 drone2_dock_x:=0.5 drone2_dock_y:=0.5"

new_window "dashboard" \
  "sleep $DELAY_APPS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_dashboard dashboard.launch.py drone1_id:=drone1 drone2_id:=drone2 port:=8080"

new_window "parcel_viz" \
  "sleep $DELAY_APPS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_parcel_visualizer parcel_visualizer.launch.py drone1_id:=drone1 drone2_id:=drone2"

new_window "customer_portal" \
  "sleep $DELAY_APPS; $DEACTIVATE; $ROS_SETUP; ros2 launch aerorelay_customer_portal customer_portal.launch.py port:=8090"

new_window "monitor" \
  "sleep 30; watch -n 2 'ros2 node list 2>/dev/null | grep -E \"drone_controller|mission_manager|battery_manager|docking_manager\"'"

tmux select-window -t "$SESSION:gazebo"
tmux attach -t "$SESSION"
