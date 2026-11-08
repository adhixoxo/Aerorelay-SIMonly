# DroneState.msg
# Full status report published periodically by each drone_controller node.
# The hub subscribes to this to track fleet status and make assignment decisions.

# --- Drone state machine constants (see project state machine spec) ---
string STATE_IDLE=IDLE
string STATE_READY=READY
string STATE_TAKEOFF=TAKEOFF
string STATE_NAVIGATE=NAVIGATE
string STATE_LAND=LAND
string STATE_WAITING=WAITING
string STATE_RETURN=RETURN
string STATE_CHARGING=CHARGING

# --- Identity ---
string drone_id                # e.g. "drone_1"

# --- State machine ---
string state                    # one of the STATE_* constants above
string current_mission_id       # empty string if no active mission

# --- Telemetry ---
float32 battery_percent         # 0.0 - 100.0
geometry_msgs/Point position    # local ENU position (x, y, z in meters)
float64 latitude
float64 longitude
float64 altitude

# --- Health ---
bool healthy                    # false if a fault/failsafe is active
string health_message           # human-readable detail, empty if healthy

builtin_interfaces/Time stamp
