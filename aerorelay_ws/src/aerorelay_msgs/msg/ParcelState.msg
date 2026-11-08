# ParcelState.msg
# Tracks a single parcel through its full relay lifecycle.
# Published by parcel_manager whenever the parcel's state changes.

# --- Parcel state machine constants (see project state machine spec) ---
string STATE_WAITING=WAITING
string STATE_ASSIGNED=ASSIGNED
string STATE_PICKED_UP=PICKED_UP
string STATE_AT_HUB=AT_HUB
string STATE_ON_DRONE_2=ON_DRONE_2
string STATE_DELIVERED=DELIVERED
string STATE_COMPLETED=COMPLETED

string parcel_id
string state                     # one of the STATE_* constants above

string assigned_drone_id         # empty if unassigned
string mission_id                # links back to the MissionRequest that created this parcel

geometry_msgs/Point pickup_location
geometry_msgs/Point delivery_location

builtin_interfaces/Time stamp
