# HubStatus.msg
# Published by mission_manager to report the hub's own orchestration state.

string STATE_IDLE=IDLE
string STATE_MISSION_RECEIVED=MISSION_RECEIVED
string STATE_ASSIGN_DRONE=ASSIGN_DRONE
string STATE_WAIT_FOR_RETURN=WAIT_FOR_RETURN
string STATE_TRANSFER_PACKAGE=TRANSFER_PACKAGE
string STATE_ASSIGN_SECOND_DRONE=ASSIGN_SECOND_DRONE
string STATE_WAIT_FOR_DELIVERY=WAIT_FOR_DELIVERY
string STATE_MISSION_COMPLETE=MISSION_COMPLETE

string state
string active_mission_id
string drone1_id
string drone2_id

builtin_interfaces/Time stamp
