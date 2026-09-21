# Every object's visibility, published together on a timer.
#
# One message rather than one topic per object: a task that has to decide
# WHICH object to go and look at needs to compare them, and three
# subscriptions that arrive at three different instants cannot be compared.
#
# The header stamp is when the snapshot was taken, on the node's own clock --
# not a detection time. Per-camera detection ages are in each CameraVisibility.
std_msgs/Header header
ObjectVisibility[] objects
