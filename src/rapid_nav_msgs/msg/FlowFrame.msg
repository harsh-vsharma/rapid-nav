# Dense optical flow between two consecutive camera frames, with what the next stages need.
std_msgs/Header header                 # stamp of the current (second) frame; frame_id camera_optical_frame
builtin_interfaces/Time previous_stamp # stamp of the previous (first) frame
sensor_msgs/Image flow                 # 32FC2, pixels, previous -> current, at the processing size
sensor_msgs/Image depth                # 32FC1, metres, current frame, same size (nearest-neighbour resize)
sensor_msgs/CameraInfo camera_info     # intrinsics scaled to the processing size
string backend                         # the backend that actually ran: farneback, raft_small, raft_large
float32 latency_ms                     # wall-clock compute time of the flow alone
