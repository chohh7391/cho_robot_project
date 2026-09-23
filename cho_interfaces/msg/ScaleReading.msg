# One weight reading, in the shape a controller needs it.
#
# Deliberately NOT hansung_scale_msgs/WeightStamped. cho_sensor/hansung_scale is
# a self-contained module that has to stay usable outside this repository, so
# nothing here depends on it; a relay adapts the two
# (cho_control_tools scale_relay).
#
# Equally deliberately NOT a bare Float64, which is what the pouring controller
# read before: `stable` is the settle signal and a plain number cannot carry it.
# Measured on the HS-AA, during flow the indicator reports US on 96% of changing
# samples and returns to ST about 1.0 s after flow stops.
#
# header.stamp is carried for bags and logs. The pouring controller does NOT
# time samples by it: it stamps arrival on its own clock, because every window
# it measures is against the controller_manager's time, and header.stamp can be
# on another clock entirely (wall time under a sim-time bringup, another PC's).
# The HS-AA driver stamps after parsing, so the two differ by well under a
# control cycle anyway.
#
# Read `stable` only when the caller already knows the flow is stopped. It is a
# statement about the indicator's own motion detector, not about the pour: at
# drip rates the same HS-AA sat still and reported ST for 5.4 s with material
# still arriving.

std_msgs/Header header

# Mass in grams. Absolute -- this scale cannot be tared over RS232, so the
# reading includes whatever vessel is on the pan.
float64 grams

# True when the indicator reports a settled reading.
bool stable
