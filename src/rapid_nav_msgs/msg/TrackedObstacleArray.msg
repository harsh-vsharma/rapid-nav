# All obstacles a velocity source tracks at one time.
std_msgs/Header header          # stamp: when the positions are valid; frame_id: map
string source                   # e.g. oracle, oracle_static, flow, fused
TrackedObstacle[] obstacles
