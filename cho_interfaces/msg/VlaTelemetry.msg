# Diagnostics for one VLA controller's chunk pipeline.
#
# Published by the controller, consumed by bridges and by the operator. The
# fields a bridge actually needs to steer itself are remaining_horizon_sec (when
# to send the next observation) and playback_stamp (where to cut its RTC
# prev_chunk_left_over prefix).
std_msgs/Header header

string stream_state          # waiting_first_chunk | running | hold | aborted
string action_space          # joint | task

uint64 chunks_accepted
uint64 chunks_rejected
# Waypoints thrown away because their execution time had already passed on
# arrival. Persistently nonzero means inference latency exceeds the chunk
# horizon: the policy is planning into a window that is already gone.
uint64 waypoints_dropped_past
string last_reject           # reject reason name, "none" when nothing was refused

# t_arr - t_obs of the last accepted chunk [s].
float64 last_chunk_latency
# Observation stamp of the last accepted chunk, and the absolute time the
# sampler last produced a reference for. Both are on the controller clock.
float64 last_chunk_stamp
float64 playback_stamp
float64 remaining_horizon_sec
uint32 queue_depth
# True when the observation-time anchor for the last accepted chunk had to fall
# back to the oldest or newest entry in the reference ring instead of an exact
# lookup. A steady stream of these means the ring is too short for the measured
# latency.
bool anchor_inexact
