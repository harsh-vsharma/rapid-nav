# One obstacle as a velocity source sees it, in the frame of the array's header.
string id                       # stable across messages for the same obstacle
geometry_msgs/Point position    # centre (m); z unused
geometry_msgs/Vector3 velocity  # m/s in the header frame; z unused
float64 radius                  # radius of a circle that covers the obstacle's outline (m)
float32 confidence              # 0..1; the oracle publishes 1.0
float32 position_sigma          # 1-sigma position error per axis (m); 0 = unknown (the oracle publishes 0)
float32 velocity_sigma          # 1-sigma velocity error per axis (m/s); 0 = unknown
