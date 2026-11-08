# AeroRelay

**Autonomous Multi-Drone Relay Delivery System**

AeroRelay is a ROS 2-based multi-drone delivery project exploring autonomous mission coordination, precision landing, docking stations and relay-based package delivery.

## Project overview

The current simulation prototype uses:
- Two ArduCopter SITL drones.
- Gazebo Harmonic for the simulated environment.
- ROS 2 Jazzy for communication and coordination.
- MAVROS for flight-controller communication.
- AprilTag-based detection and precision landing.
- A central mission manager and docking manager.
- Battery monitoring, parcel visualization and web dashboards.

The intended physical prototype includes autonomous docking, package transfer between drones and cable-based last-meter delivery. These are development goals; simulation features and physical implementation should not be assumed equivalent.

## Repository structure

- `aerorelay_ws/src/` — ROS 2 packages
- `aerorelay_ws/mav.parm` — SITL parameters
- `aerorelay_ws/mav_2_1.parm` — SITL parameters
- `aerorelay_ws/start_all.sh` — Start simulation stack
- `aerorelay_ws/stop_all.sh` — Stop simulation stack
- `aerorelay_ws/restart_sitl.sh` — Restart SITL drones
- `ardupilot/` — ArduPilot Git submodule
- `ardupilot_gazebo/` — Gazebo plugin, models and worlds
- `tools/` — Development utilities
- `requirements.txt` — Python dependencies

## Requirements

The development environment is based on:

- Ubuntu with ROS 2 Jazzy.
- Gazebo Harmonic.
- ArduPilot SITL.
- ArduPilot Gazebo plugin.
- MAVROS and MAVProxy.
- `tmux` and `taskset`.
- Python 3.12 and the packages listed in `requirements.txt`.

## Setup

Clone the repository with its ArduPilot submodule:

    git clone --recurse-submodules <repository-url> ~/AeroRelay
    cd ~/AeroRelay

If the submodule was not initialized:

    git submodule update --init --recursive

Build the ROS 2 workspace:

    cd ~/AeroRelay/aerorelay_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install

Install the Python dependencies:

    cd ~/AeroRelay
    python3.12 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt

## Running the simulation

Start the complete simulation stack:

    cd ~/AeroRelay/aerorelay_ws
    ./start_all.sh

Stop the simulation stack:

    ./stop_all.sh

Restart both SITL drones:

    ./restart_sitl.sh

The launcher assumes the required ROS 2, Gazebo, ArduPilot, MAVROS and MAVProxy components have already been installed and configured.

## Development status

AeroRelay is an ongoing development project. The simulation environment and individual software modules are being developed and integrated. Physical drone construction and complete autonomous delivery remain part of the planned implementation.
