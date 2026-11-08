# DockPosition.msg
# Published periodically by docking_manager for each drone's assigned dock.
# mission_manager uses this as the RETURN target; battery_manager uses it
# to detect "docked" for charging.

string drone_id
float64 x
float64 y
float64 z
float64 dock_radius   # meters; how close counts as "docked"
