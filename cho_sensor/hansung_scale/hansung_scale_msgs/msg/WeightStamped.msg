# One weight reading from an RS232 scale/indicator.
#
# Mirrors the stamped, self-describing sample shape realsense2_camera_msgs
# uses, so a consumer never has to time-correlate several primitive topics
# (~/weight, ~/stable, ~/unit) that were published as separate messages.

# Raw 2-character status field from the indicator frame.
# ST is confirmed on the HS-AA; US and OL follow the convention for this
# frame style but have not been observed on real hardware yet.
string STATUS_STABLE=ST
string STATUS_UNSTABLE=US
string STATUS_OVERLOAD=OL

std_msgs/Header header

# Value exactly as the indicator reported it, with the frame sign applied.
float64 weight
# Unit string as the indicator reported it, for example g, kg or lb.
string unit
# The weight field converted to grams, or NaN when the unit is not one we know.
float64 weight_grams

# True only when the indicator reports a settled reading (status is ST).
bool stable
string status
